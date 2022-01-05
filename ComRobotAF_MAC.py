# -*- coding: utf-8 -*-

"""
       .==.        .==.
      //`^\\      //^`\\
     // ^ ^\(\__/)/^ ^^\\
    //^ ^^ ^/+  0\ ^^ ^ \\
   //^ ^^ ^/( >< )\^ ^ ^ \\
  // ^^ ^/\| v''v |/\^ ^ ^\\
 // ^^/\/ /  `~~`  \ \/\^ ^\\
 ----------------------------
BE CAREFULL! THERE IS A DRAGON.

功能：机器人多址接入协议功能验证模块
备注：

案例：

模块：
(c) 黄吉 2021
依赖：
"""
import copy
import math
import random

isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Communication.simulation.ComRobotAF import ComRobotAF
from Communication.simulation.ComRobot import ComRobot
from Communication.simulation.ComObjectCollection import *
from Communication.simulation.ComObject import ComObject


class VLCMsg:
    def __init__(self, sourceId, targetId=-1, targetBox=-1, isBroadcasting=False, isFlooding=False):
        self.sourceId = sourceId

        self.targetId = targetId
        self.targetBox = targetBox
        self.linkPath = []

        self.isBroadcasting = isBroadcasting

        self.isFlooding = isFlooding
        self.seq = -1
        self.relayId = -1

        self.isSetTransLimit = False
        self.transLimit = -1
        self.transNum = 0

        self.payload = {}


# 废弃
class ComRobotAF_MAC(ComRobotAF):
    def __init__(self, pos, stage):
        super().__init__(pos)
        self.mCommunicationRange = 400
        self.mSenseDistance = 200

        self.slots = set()  # 每个机器人占用的槽索引
        self.status = 'entering'  # 几种可能：online-pre-在群体中以预备槽工作，online-在群体中正常工作，entering/leaving-机器人在进入或者离开群体

        self.stage = stage  # ComStage_MAC对象

        self.in_robots = []  # 入邻居
        self.out_robots = []  # 出邻居
        self.robots_inRs = []  # 感知范围内的所有机器人

        self.idsNeedKnow = []  # 记录我们机器人需要在知道的其他机器人信息的id：[‘ALL’]表示全地图都要掌握

        self.sendMsg = []  # 发送槽信息集合,发送缓存

        self.resFlag = 0
        self.refResDict = {}
        self.resHistory = set()

        self.hasSendReqFlag = 0  # 机器人准备接入网络时发送一次请求信息
        self.hasSendLeaFlag = 0  # 机器人离开网络发送最后消息包，0未离开，1离开未发送，2离开已发送
        self.hasSlotChangedFlag = 0  # 机器人的槽信息变化了，向网络中洪泛。

        self.collisionMsg = {'flag': 0, 'msg': []}

        self.globalAG = np.mat(np.zeros((60, 60)))  # 每个机器人维护一个全局的表

        self.acsInfo = []  # 接收信息存放，raw data，接收缓存
        self.p_acsInfo = {  # 处理过后的存储在机器人本地的信息
            self.mId: {'type': '', 'slots': copy.deepcopy(self.slots)},
        }
        self.p_acsInfoRecorder = []  # 记录历史值

        self.floodSeq = 0  # 记录当前已经发了多少个洪泛包了
        self.floodHistory = {}  # 记录接收到的洪泛包的源id与seq值，防止循环洪泛。

        self.mObjectType = "ComRobotAF_MAC"  # 用于标识当前物体类别

        # 以下是测试指标所需的一些变量
        self.startAccessFlag = 0  # 开始接入
        self.accessCount = 0  # 接入所花费的时间
        self.endAccessFlag = 0  # 接入成功

        # 以下变量为 布置机器人位置时所用，为了方便而写的，没啥大用
        self.initDirection = 0.0
        self.flag1 = 0
        self.randomInFlag = 0

    def refresh(self):
        self.robots_inRs.clear()
        self.in_robots.clear()
        self.out_robots.clear()
        self.acsInfo.clear()

    def communicateTo(self, robot, nowSlot):
        if self.isCommunicating and (nowSlot in self.slots):
            robot.acsInfo.extend(self.sendMsg)
            robot.mInfo.append(self.mProcessedInfo)
            return len(self.sendMsg)
        return 0

    def sense(self):
        # 这里光通信机器人通信范围为扇形，而感知是360度的，与师兄的实现刚好是相反的，那我们就反着用就好了
        obj_in_sense_length = getObjectInRange(self.mPos, self.mCommunicationRange)  # 感知距离以内的机器人
        obj_in_sense_length.remove(self)  # 排除自己

        if len(obj_in_sense_length) == 0:
            return

        self.robots_inRs = obj_in_sense_length
        if self.stage.macProtocol == 'C-CMSTDMA':
            self.idsNeedKnow = [obj.mID for obj in obj_in_sense_length]
        else:
            self.idsNeedKnow = ['ALL']

        obj_in_comm_length = getObjectInRange(self.mPos, self.mSenseDistance)  # 通信距离以内的机器人
        obj_in_comm_length.remove(self)  # 排除自己

        if len(obj_in_comm_length) == 0:
            return

        # 挑选出夹角以内的机器人，即xy平面内的偏航角度正负self.mSenseAngle以内，与xy平面的夹角正负self.mSenseAngle以内
        for robot in obj_in_comm_length:
            angle_in_xy = ComObject.getAngleBetweenXandVector(self.mPos, robot.mPos, plat='xy') - self.mDirection  # xy平面内的偏航角度
            angle_with_xy = ComObject.getAngleBetweenXandVector(robot.mPos, self.mPos, plat='o-xy')  # 与xy平面的夹角
            if angle_in_xy > math.pi:
                angle_in_xy = 2*math.pi - angle_in_xy
            if angle_in_xy < -math.pi:
                angle_in_xy = 2*math.pi + angle_in_xy
            if (-self.mSenseAngle/2) <= angle_in_xy <= (self.mSenseAngle/2) and \
                    (-self.mSenseAngle/2) <= angle_with_xy <= (self.mSenseAngle/2):
                self.mProcessedInfo[robot.mObjectType][robot.mId] = copy.deepcopy(robot.mPos)
                self.out_robots.append(robot)

    '''*********************数据接收和解析部分**************************************'''

    def getNetworkInfo(self):  # 机器人从准备状态转换到online时，入机器人会把网络信息发送给它。
        if len(self.in_robots) < 1:
            print('robot:', self.mId, ' has no in neighbor robot, can not get the network information.')
            return

        robot = None
        for rbt in self.in_robots:
            if rbt.status == 'online':
                robot = rbt

        if robot is None:
            print('robot:', self.mId, ' has no online in neighbor robot, can not get the network information.')
            return

        self.p_acsInfo = copy.deepcopy(robot.p_acsInfo)
        self.globalAG = copy.deepcopy(robot.globalAG)
        self.collisionMsg = copy.deepcopy(robot.collisionMsg)

        # 因为上面深拷贝的邻居的信息，这里把自己的信息还原一下。
        # 更新自己的邻居
        self.globalAG[:, self.mId] = 0
        innbs = [rbt.mId for rbt in self.in_robots]
        for innb in innbs:
            self.globalAG[innb, self.mId] = 1

        self.p_acsInfo[self.mId] = {'type': '', 'slots': copy.deepcopy(self.slots)}

    def updateSlots(self):
        if self.status == 'entering':
            return

        # if self.status == 'leaving':
        #     for s in self.slots:
        #         self.stage.rbtID_Time[s].remove(self.mId)
        #     self.slots.clear()
        #     return

        if self.status == 'online-pre':
            if not self.slots and not self.p_acsInfo[self.mId]['slots']:
                s = random.randint(1, self.stage.preSlots) + self.stage.formerSlots - 1
                self.slots.add(s)
                self.stage.rbtID_Time[s].add(self.mId)

            elif self.slots and not self.p_acsInfo[self.mId]['slots']:  # 如果机器人还没有正式接入，防止多个机器人同时占用同一个预备slot，咱们每一帧都换一个perslot发送
                if len(self.slots) == 1 and self.stage.nowTime == 19 and list(self.slots)[0] in range(self.stage.formerSlots, self.stage.formerSlots + self.stage.preSlots):
                    s = list(self.slots)[0]
                    self.slots.discard(s)
                    self.stage.rbtID_Time[s].discard(self.mId)
                    s = random.randint(1, self.stage.preSlots) + self.stage.formerSlots - 1
                    self.slots.add(s)
                    self.stage.rbtID_Time[s].add(self.mId)

                    self.hasSendReqFlag = 0

            elif not self.slots and self.p_acsInfo[self.mId]['slots']:  # 从map中获取的，暂时用不上、
                for s in self.p_acsInfo[self.mId]['slots']:
                    self.slots.add(s)
                    self.stage.rbtID_Time[s].add(self.mId)
                self.status = 'online'
                self.hasSlotChangedFlag = 1

                self.hasSendReqFlag = 1
                self.endAccessFlag = 1  # 接入成功了
                self.startAccessFlag = 0
                print('*****new robot stop accessing***********')

            elif self.slots and self.p_acsInfo[self.mId]['slots']:
                if not (len(self.slots) == 1 and list(self.slots)[0] in range(self.stage.formerSlots, self.stage.formerSlots + self.stage.preSlots)):
                    self.hasSlotChangedFlag = 1
                for s in self.slots:
                    self.stage.rbtID_Time[s].remove(self.mId)
                self.slots.clear()
                for s in self.p_acsInfo[self.mId]['slots']:
                    self.slots.add(s)
                    self.stage.rbtID_Time[s].add(self.mId)

                self.status = 'online'

                self.hasSendReqFlag = 1
                self.endAccessFlag = 1  # 接入成功了
                self.startAccessFlag = 0
                print('*****new robot stop accessing***********')

            else:
                return

        if self.status == 'online':
            if self.slots == self.p_acsInfo[self.mId]['slots']:
                return
            else:
                for s in self.slots:
                    self.stage.rbtID_Time[s].remove(self.mId)
                self.slots = copy.deepcopy(self.p_acsInfo[self.mId]['slots'])
                for s in self.slots:
                    self.stage.rbtID_Time[s].add(self.mId)

    def updatePacsInfo(self, payload: dict):
        if 'REQ' in payload.keys():
            if self.status == 'online-pre':
                return

            sid = payload['REQ'][0]
            if sid not in self.resHistory:
                print('id:', self.mId, 'received a request msg from id:', payload['REQ'][0])
                tarr = copy.deepcopy(payload['REQ'][1])
                self.resFlag = 1
                self.refResDict[sid] = tarr
            return

        if 'NEIGHBORS' in payload.keys():
            # print('id:', self.mId, 'received a in neighbors msg from id:', payload['NEIGHBORS'][0])

            sid = payload['NEIGHBORS'][0]
            innbs = copy.deepcopy(payload['NEIGHBORS'][1])
            self.globalAG[:, sid] = 0
            for innb in innbs:
                self.globalAG[innb, sid] = 1
            return

        if 'COLLISION' in payload.keys():
            for item in payload['COLLISION']:
                if item not in self.collisionMsg['msg']:
                    self.collisionMsg['msg'].append(item)
            return

        # print('id:', self.mId, 'received a in slot msg:', payload)

        for id, val in payload.items():
            if val['type'] == 'slot-add':
                if id not in self.p_acsInfo.keys():
                    self.p_acsInfo[id] = {'type': '', 'slots': set()}
                for s in val['slots']:
                    self.p_acsInfo[id]['slots'].add(s)
            elif val['type'] == 'slot-rdu':
                if id not in self.p_acsInfo.keys():
                    self.p_acsInfo[id] = {'type': '', 'slots': set()}
                for s in val['slots']:
                    self.p_acsInfo[id]['slots'].discard(s)
            elif val['type'] == 'slot-rpl':
                self.p_acsInfo[id] = copy.deepcopy(val)
            else:
                print('VLCMsgError:: msg has no type, please check...')

    def decoderVLCMsg(self, msg, dcType):
        if dcType == 'Broadcasting':
            self.updatePacsInfo(msg.payload)
        if dcType == 'Flooding':
            if msg.isSetTransLimit and (msg.transNum > msg.transLimit):  # 如果洪泛包寿命已经超过极限，误传播，虽然理论上不可能，防止情况发生
                return
            if msg.sourceId == self.mId:  # 如果是自己发送的数据包又回来了
                # print('id:', self.mId, 'receive a cycle msg:', msg.payload)
                return

            if self.floodHistory.get(msg.sourceId) is None:  # 历史记录表示没有接收过来自该id的数据包
                self.floodHistory[msg.sourceId] = list(range(msg.seq + 1))  # 记录该id的数据包的seq值，以后再接收到来自该id的seq数据包就忽视
                self.updatePacsInfo(msg.payload)
                self.sendMsg.append(msg)  # 接收完成之后传到发送缓存中，准备进行转发
            else:
                '''
                举例：floodHistory[msg.sourceId] = [2,3,6]
                说明：1、数组中的最大值6表示已经接收过来自msg.sourceId的seq为6的数据包了，
                      2、数组中小于6的值2，3表示还没接收到来自msg.sourceId的seq为2和3的数据包，
                      3、所有小于6但又没出现在数组中值，这里的0，1，4，5表示已经接收过来自msg.sourceId的seq为6的数据包了，
                         这里隐去是为了节约存储空间。
                      4、所有大于6但又没出现在数组中值，这里的7，8，9....表示还没接收过来自msg.sourceId的seq为7，8，9....的数据包
                '''
                maxhist = max(self.floodHistory[msg.sourceId])
                if msg.seq in self.floodHistory[msg.sourceId]:           # 当前数据包seq值在floodHistory中
                    if msg.seq == maxhist:  # 注释情况1
                        # print('id:', self.mId, 'has received this msg: ', msg.payload, ' with case 1')
                        pass                                             # 已经接收过了，就没必要重复接收转发了，防止广播瀑布
                    else:                                                # 注释情况2
                        # print('id:', self.mId, 'receive a msg: ', msg.payload, ' with case 2')
                        self.floodHistory[msg.sourceId].remove(msg.seq)
                        self.updatePacsInfo(msg.payload)
                        if msg.isSetTransLimit:  # 如果消息设置了洪范寿命
                            if msg.transNum == msg.transLimit:
                                return
                            else:
                                msg2 = copy.deepcopy(msg)
                                msg2.transNum += 1
                                self.sendMsg.append(msg2)
                        else:
                            self.sendMsg.append(msg)
                else:
                    if msg.seq < maxhist:  # 注释情况3
                        # print('id:', self.mId, 'has received this msg: ', msg.payload, ' with case 3')
                        pass                                            # 已经接收过了，就没必要重复接收转发了，防止广播瀑布
                    else:                                               # 注释情况4
                        # print('id:', self.mId, 'receive a msg: ', msg.payload, ' with case 4')
                        self.floodHistory[msg.sourceId].remove(maxhist)
                        for i in range(maxhist + 1, msg.seq + 1):
                            self.floodHistory[msg.sourceId].append(i)

                        self.updatePacsInfo(msg.payload)
                        if msg.isSetTransLimit:
                            if msg.transNum == msg.transLimit:
                                return
                            else:
                                msg2 = copy.deepcopy(msg)
                                msg2.transNum += 1
                                self.sendMsg.append(msg2)
                        else:
                            self.sendMsg.append(msg)

    def updateAcsInfo(self):  # 对接收缓存acsInfo中的数据（VLCMsg）进行处理
        if self.status == 'entering' or self.status == 'leaving':
            return

        for info in self.acsInfo:
            if info.isFlooding:  # 如果是洪泛数据包：解包，判断，转发
                self.decoderVLCMsg(info, 'Flooding')
            elif info.isBroadcasting:  # 不是洪泛包，但是是广播包，表示是传给邻居的包：解包，判断
                self.decoderVLCMsg(info, 'Broadcasting')
            else:   # 非洪泛，非广播，定向数据包
                if info.targetId == self.mId:  # 传给我的定向数据包：解包。
                    pass
                elif self.mId == info.linkPath[0]:  # 只是经过我这里：顺着链路传递。
                    pass
                else:  # 无意中接收到的定向包，不经过我这里：抛弃。
                    pass
    '''*********************end*****************************************************'''

    '''*********************生成发送数据包部分**************************************'''

    def chooseSlot(self, key, ids):  # ids是新接入机器人的出邻居id数组
        csr = set(range(self.stage.formerSlots))
        sNum = {}  # 储存所有机器人v的id和他们的slots数量
        sFeq = {}  # 储存所有机器人v一起占用的slot使用频率
        for rid in ids:  # 每个rid表示能被id=key的机器人照射到的机器人id。
            var1 = np.nonzero(self.globalAG[:, rid])[0]  # var1数组表示id=rid机器人的入邻居id数组,,
            if key in var1:
                var1 = np.setdiff1d(var1, [key])  # 注意要剔除自己的id

            for v in var1:
                if v in self.p_acsInfo:
                    csr -= self.p_acsInfo[v]['slots']

                    if v not in sNum:
                        sNum[v] = len(self.p_acsInfo[v]['slots'])
                        for ts in self.p_acsInfo[v]['slots']:
                            if ts not in sFeq:
                                sFeq[ts] = 1
                            else:
                                sFeq[ts] += 1

                else:  # 如果id=v的机器人不在自己的p_acsInfo中记录，表示id=v的机器人slot信息还没有同步到我这里来，就是我忽略一个机器人的信息。
                    print('id:', self.mId, ' because i do not know the slot about robot:', v, 'so i ignore the difference among:', {key, v})
                    self.collisionMsg['flag'] = 1
                    s = {key, v}
                    flag = 0
                    for item in self.collisionMsg['msg']:
                        if s.issubset(item):
                            flag = 1
                            break
                    if not flag:
                        self.collisionMsg['msg'].append(s)
        s = set()
        if len(csr) > 0:
            s.add(random.choice(list(csr)))
            return s

        slen = round(len(ids)/2)
        ids = sorted(ids)
        if self.mId not in [ids[i] for i in range(slen)]:
            return s

        tid = list(sNum.keys())[list(sNum.values()).index(max(list(sNum.values())))]  # 找出slot最多的那个机器人id=tid
        if sNum[tid] == 1:
            print('!!!!!!!!!!!!!!!!!!!!!every robot has only one slot, new robot has no choice!!!!!!!!!!!!!!')
        else:
            # 找出id=tid这个机器人的slot中，被占用次数最少的那个:ts
            tarr = [(ts, sFeq[ts]) for ts in self.p_acsInfo[tid]['slots']]
            ts = tarr[[item[1] for item in tarr].index(min([item[1] for item in tarr]))][0]
            s.add(ts)
            # 发送信息：把id=tid的机器人ts槽给取消掉，给新机器人用
            chg = VLCMsg(self.mId, isFlooding=True)
            chg.seq = self.floodSeq
            for rid in ids:  # 每个rid表示能被id=key的机器人照射到的机器人id。
                var1 = np.nonzero(self.globalAG[:, rid])[0]  # var1数组表示id=rid机器人的入邻居
                for v in var1:
                    if v in self.p_acsInfo and ts in self.p_acsInfo[v]['slots']:
                        chg.payload[v] = {'type': 'slot-rdu', 'slots': s}
            self.sendMsg.append(chg)
            self.floodSeq += 1
            self.p_acsInfo[tid]['slots'].discard(ts)

            print('@@@@@@@@@@@@@@@@ reduce robot:', tid, ' with slot: ', ts, ' to guarantee new robot in @@@@@@@@@')
        return s

    def genVLCMsg(self):
        if self.status == 'entering':
            return

        if self.status == 'leaving':
            lea = VLCMsg(self.mId, isFlooding=True)
            lea.seq = self.floodSeq
            self.floodSeq += 1
            lea.payload['LEAVING'] = self.mId
            self.sendMsg.append(lea)
            self.hasSendLeaFlag = 1

        if self.status == 'online-pre':
            # 以Broadcasting方式向出邻居发送request请求。
            if not self.hasSendReqFlag and (len(self.out_robots) == len([robot for robot in self.out_robots if robot.status in ['online-pre', 'online']])):
                print('id:', self.mId, ' status: pre slot:', self.slots, 'gen request msg---with out:', [robot.mId for robot in self.out_robots], ' out-online:', [robot.mId for robot in self.out_robots if robot.status == 'online'], ' out-online-pre:', [robot.mId for robot in self.out_robots if robot.status == 'online-pre'])
                # print('id:', self.mId, ' status: pre slot:', self.slots, 'gen in neighbors flood msg:',
                #      [robot.mId for robot in self.in_robots])

                req1 = VLCMsg(self.mId, isBroadcasting=True)
                req1.payload['REQ'] = [self.mId, [robot.mId for robot in self.out_robots]]
                req2 = VLCMsg(self.mId, isFlooding=True)
                req2.seq = self.floodSeq
                self.floodSeq += 1
                req2.payload['NEIGHBORS'] = [self.mId, [robot.mId for robot in self.in_robots]]
                self.sendMsg.append(req1)
                self.sendMsg.append(req2)

                self.hasSendReqFlag = 1

        if self.status == 'online':
            if self.resFlag == 1:
                for key, ids in self.refResDict.items():  # key就是新接入的机器人id
                    s = self.chooseSlot(key, ids)
                    if not s:
                        continue
                    res = VLCMsg(self.mId, isFlooding=True)
                    res.seq = self.floodSeq
                    res.payload[key] = {'type': 'slot-add', 'slots': s}
                    self.sendMsg.append(res)
                    self.floodSeq += 1
                    self.p_acsInfo[key] = {'type': '', 'slots': s}

                    self.resHistory.add(key)

                    print('id:', self.mId, ' status: online, gen response msg for:', key, 'with slot:', s)

                if len(self.refResDict.keys()) > 1:  # 如果要response列表大于1，说明有多个机器人同时照到我了，二者要槽信息不同
                    self.collisionMsg['flag'] = 1
                    flag = 0
                    s = set(self.refResDict.keys())
                    for item in self.collisionMsg['msg']:
                        if s.issubset(item):
                            flag = 1
                            break
                    if not flag:
                        self.collisionMsg['msg'].append(s)

                self.refResDict.clear()

                if self.collisionMsg['flag'] == 1:  # 发送接入冲突消息包
                    coll = VLCMsg(self.mId, isFlooding=True)
                    coll.seq = self.floodSeq
                    self.floodSeq += 1
                    coll.payload['COLLISION'] = copy.deepcopy(self.collisionMsg['msg'])
                    self.sendMsg.append(coll)

                    self.collisionMsg['flag'] = 0

                res2 = VLCMsg(self.mId, isFlooding=True)
                res2.seq = self.floodSeq
                self.floodSeq += 1
                res2.payload['NEIGHBORS'] = [self.mId, [robot.mId for robot in self.in_robots]]
                self.sendMsg.append(res2)

                self.resFlag = 0

            if self.hasSlotChangedFlag == 1:
                scpack = VLCMsg(self.mId, isFlooding=True)
                scpack.seq = self.floodSeq
                self.floodSeq += 1
                scpack.payload[self.mId] = {'type': 'slot-rpl', 'slots': copy.deepcopy(self.slots)}
                self.sendMsg.append(scpack)
                self.hasSlotChangedFlag = 0

        # 检测有没有merging collision
        # self.collisionDetect()
    '''****************************************************end*****************************************************'''

    '''*************************************碰撞（冲突）检测相关代码****************************************************'''

    def inCollisionDetect(self):  # 每个机器人都要根据collisionMsg变量更新p_acsInfo中的冲突问题，策略就是如果真的存在冲突，就谁的slot数量多就谁删除冲突的槽。
        ret = set()
        for arr in self.collisionMsg['msg']:   # arr就是一个互斥数组
            muxDict = {mId: copy.deepcopy(self.p_acsInfo[mId]['slots']) for mId in arr if mId in self.p_acsInfo}
            ids = list(muxDict.keys())
            if len(ids) < 2:
                return ret
            s1 = muxDict[ids[0]]  # s1到最后就是arr互斥数组机器人之间的slot有没有存在冲突，有就不为空，为空就说明没有冲突
            for vals in muxDict.values():
                s1 = s1 & vals

            for s in s1:
                muxDict2 = [(mId, len(vals)) for mId, vals in muxDict.items() if s in vals]
                mIds = [item[0] for item in muxDict2]
                lens = [item[1] for item in muxDict2]
                maxLenId = mIds[lens.index(max(lens))]

                if maxLenId == self.mId:  # 如果是自己需要删除某个slot，就记录，返回，如果不是自己就pass
                    self.slots.discard(s)
                    ret.add((self.mId, s))

                muxDict[maxLenId].discard(s)  # 必要的一步， 不然可能重复删除某一个id的机器人的slots。

        return ret

    def mergeCollisionDetect(self):
        for robot in self.in_robots:
            outIds = np.nonzero(self.globalAG[robot.mId, :])[0]  # var1数组表示机器人的出邻居id数组,
            for rid in outIds:
                var1 = np.nonzero(self.globalAG[:, rid])[0]  # var1数组表示id=rid机器人的入邻居id数组,,
                if robot.mId in var1:
                    var1 = np.setdiff1d(var1, [robot.mId])  # 注意要剔除自己的id

                for v in var1:  # 那么每个id=v的机器人应该与id=robot.mId的机器人没有slot交集
                    if v in self.p_acsInfo:
                        s = robot.slots & self.p_acsInfo[v]['slots']

                        if s:
                            flag = 0
                            for item in self.collisionMsg['msg']:
                                if s.issubset(item):
                                    flag = 1
                                    break
                            if not flag:
                                self.collisionMsg['flag'] = 1
                                self.collisionMsg['msg'].append({robot.mId, v})

                                print(self.mId, 'rid: ', rid, '---merging collision detection: find collision__', {robot.mId, v})
                                print('outIds:', outIds)
                                print('var1:', var1)

                    else:  # 如果id=v的机器人不在自己的p_acsInfo中记录，表示id=v的机器人slot信息还没有同步到我这里来，就是我忽略一个机器人的信息。
                        s = {robot.mId, v}
                        flag = 0
                        for item in self.collisionMsg['msg']:
                            if s.issubset(item):
                                flag = 1
                                break
                        if not flag:
                            self.collisionMsg['flag'] = 1
                            self.collisionMsg['msg'].append(s)

                            print('id:', self.mId, 'rid:', rid, ' because i do not know the slot about robot:', v,
                                  'so i ignore the difference among:', {robot.mId, v},
                                  'in merging collision detecting function')
                            print('merging collision detection: find collision__', s)

        if self.collisionMsg['flag'] == 1:  # 发送接入冲突消息包
            coll = VLCMsg(self.mId, isFlooding=True)
            coll.seq = self.floodSeq
            self.floodSeq += 1
            coll.payload['COLLISION'] = copy.deepcopy(self.collisionMsg['msg'])
            self.sendMsg.append(coll)

            self.collisionMsg['flag'] = 0

    '''****************************************************end*****************************************************'''

    '''************************************机器人接入网络中相关代码*****************************************************'''
    def checkin(self):  # 机器人的接入条件：至少有一个online的入邻居，至少一个online的出邻居
        if len(self.out_robots) < 1 or len(self.in_robots) < 1:
            return False

        num = 0
        for rbt in self.out_robots:
            if rbt.status == 'online':
                num += 1
        if num < 1:
            return False

        num = 0
        for rbt in self.in_robots:
            if rbt.status == 'online':
                num += 1
        if num < 1:
            return False

        return True

    def calcTargetPos(self):
        if self.status == 'entering':
            if self.isStopping() and self.mPos[2] == 500:
                i = round((self.mPos[0] + self.stage.scale / 2) / self.stage.scale * self.stage.linenum - 1)
                j = round((self.mPos[1] + self.stage.scale / 2) / self.stage.scale * self.stage.linenum - 1)
                # if self.stage.map[i][j]['angle'] is not None:
                #     self.setDirection(self.stage.map[i][j]['angle'])
                #     self.status = 'online-pre'
                #     self.stage.map[i][j]['status'] = 'busy'
                #
                #     self.getNetworkInfo()
                #     self.startAccessFlag = 1
                #     print('*****new robot start accessing***********')

                if self.checkin():
                    self.status = 'online-pre'
                    if not self.randomInFlag:
                        self.stage.map[i][j]['status'] = 'busy'

                    self.getNetworkInfo()
                    self.startAccessFlag = 1
                    print('*****new robot start accessing***********')
                    print('id:', self.mId, ' in: ', [rbt.mId for rbt in self.in_robots], ' out: ', [rbt.mId for rbt in self.out_robots])
                    print('id:', self.mId, ' in-online: ', [rbt.mId for rbt in self.in_robots if rbt.status == 'online'], ' out-online: ', [rbt.mId for rbt in self.out_robots if rbt.status == 'online'])
                    print('*****************************************')

                else:
                    angle = self.mDirection + 0.54
                    if angle > math.pi:
                        angle = -math.pi
                        self.flag1 = 1
                    if (self.initDirection - angle) <= 0.54 and self.flag1 == 1:
                        self.flag1 = 0
                        if not self.randomInFlag:
                            print('???????????????????????? 发现一个无用的位置', i, j, '????????????????????????????????')
                            self.stage.map[i][j]['status'] = 'useless'
                        else:
                            print('???????????????????????? 随机接入了一个无用的位置，重新随机接入??????????????????????????')
                        range_x = (-300, 300)
                        range_y = (-300, 300)
                        x = random.uniform(range_x[0], range_x[1])
                        y = random.uniform(range_y[0], range_y[1])
                        self.mTarget = np.array([x, y, 400], dtype=np.float32)
                    else:
                        self.setDirection(angle)
            else:
                self.initDirection = self.mDirection
        # elif self.status == 'leaving':
        #     if self.hasSendLeaFlag == 2:
        #         self.mTarget = np.array([self.mPos[0],
        #                                  self.mPos[0],
        #                                  400], dtype=np.float32)
        else:
            return

    '''****************************************************end*****************************************************'''

    def update(self):
        # 计数接入时间
        if self.startAccessFlag:
            self.accessCount += 1

        # 更新自己的邻居
        self.globalAG[:, self.mId] = 0
        innbs = [rbt.mId for rbt in self.in_robots]
        for innb in innbs:
            self.globalAG[innb, self.mId] = 1

        # 处理接收到的槽信息，将信息从接收缓存acsInfo中解析到p_acsInfo中，并将需要转发的数据包添加到发送缓存sendMsg中
        self.updateAcsInfo()

        # 接收完消息之后，就对接收的消息进行一个接入冲突检测
        self.mergeCollisionDetect()

        # # 借助一致性算法更新机器人位置
        # self.processInfo()

        self.calcTargetPos()

        # 如果有必要，让机器人运动到指定位置
        self.move()

        # 将信息从p_acsInfo跟新到self.slots中，同时跟新到ComStageMAC.rbtID_Time中。
        self.updateSlots()

        # 生成发送信息
        self.genVLCMsg()

        # print(self.mId, ': acsInfo-', self.acsInfo)
        # print(self.mId, ': A(G)-', self.globalAG[0:5, 0:5])
        # print(self.mId, ': p_acsInfo-', self.p_acsInfo)

        if self.endAccessFlag:
            self.stage.accessCountHistory.append((self.mId, self.accessCount))
            self.accessCount = 0
            self.endAccessFlag = 0
            self.stage.acesSuccessFlag -= 1










