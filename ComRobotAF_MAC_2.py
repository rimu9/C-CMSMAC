# -*- coding: utf-8 -*-
import copy
import logging
import math
import multiprocessing
import random
import networkx as net
from Communication.simulation.ComObjectCollection import *
from Communication.simulation.ComObject import ComObject
from Communication.simulation.ComRobotAF import ComRobotAF
from Communication.simulation.ComRobotAF_MAC import VLCMsg


class ComRobotAFMAC2(ComRobotAF):
    def __init__(self, pos, stage):
        super().__init__(pos)
        self.mCommunicationRange = 400  # 视觉感知距离 360°
        self.mSenseDistance = 200  # 光通讯距离，有FOV

        self.slots = set()  # 每个机器人占用的槽索引
        self.status = 'entering'  # 几种可能：online-pre-在群体中以预备槽工作，online-在群体中正常工作，entering/leaving-机器人在进入或者离开群体
        self.stage = stage  # ComStage_MAC对象

        self.inRobots = []  # 入邻居
        self.outRobots = []  # 出邻居
        self.oldOutRobotsRecoder = {}  # 上一次感知出来机器人的id和状态记录，供下次感知时对比使用。
        self.robotsInRs = []  # 感知范围内的所有机器人
        self.inRobotsIdsRecoder = []  # 记录前20+1个槽中接收到消息的机器人id

        self.sendMsgBuffer = []  # 发送槽信息集合,发送缓存
        self.recvMsgBuffer = []  # 接收信息存放，raw data，接收缓存

        self.proSlotInfo = {  # 处理过后的存储在机器人本地的信息
            self.mId: {'type': '', 'slots': set()},
        }
        self.graph = net.DiGraph()  # 每个机器人对网络连接情况的了解。每个机器人发送自己的出邻居id，洪泛或者单播出去，其他机器人根据这个更新self.graph
        self.collisions = []  # [(id1, id2), (id1, id4), ...]存储接入冲突和融合冲突信息，这里面每一组表示他们之间slot不能有重复

        self.hasGetInitInfoFlag = 0  # 机器人刚转为online-pre状态时，需要经过1frame的时间然后从入邻居那里接收关于网络的一些信息。这是标志位，标志自己是否已经接收过了
        self.hasSendMsgFlag = 1  # 机器人将信息发送出去的后置1，还在发送缓存中的时候该值应该为0
        self.hasInRobotChangedFlag = 0  # 当自身检测到入邻居发生了变化，就置1，等待机器人online状态以后发生新的NEIGHBORS消息包
        self.hasOutRobotChangedFlag = 0  # # 当自身检测到出邻居发生了变化，就置1，等待机器人online状态以后发生新的NEIGHBORS消息包

        # 指标 & flag
        self.hasStartAccessFlag = 0
        self.accessCount = 0
        self.hasEndAccessFlag = 0

        self.hasResIds = set()  # 记录我已经对哪些机器人的请求做出响应了
        self.resRefInfo = {}  # {id: [outRobots ids]} 记录发出请求的机器人id以及他的出邻居，方便我们为其选取一个不冲突的slot

        self.floodSeq = 0  # 记录当前已经发了多少个洪泛包了
        self.floodHistory = {}  # 记录接收到的洪泛包的源id与seq值，防止循环洪泛。

        self.mPosIndexInMap = (-1, -1)  # 记录自己在stage.map中所占据的位置的索引
        self.initDirection = 0  # 机器人到达目标位置时，记录自己的方向，好判断自己是否转过了一圈。

        self.mObjectType = "ComRobotAFMAC2"  # 用于标识当前物体类别

    def setStartEndAccessFlag(self, start, end):
        self.hasStartAccessFlag = start
        self.hasEndAccessFlag = end

    '''
    description: 将self.slots和stage.rbtSlotsID中自己的信息进行修改更新。
    param: slots: 需要被更新到的目标
    return: none
    '''
    def changeSlots(self, slots: set):
        for s in self.slots:
            self.stage.rbtSlotsID[s].discard(self.mId)

        self.slots.clear()

        self.addSlots(slots)

    '''
    description: 将self.slots和stage.rbtSlotsID中自己的信息进行添加
    param: slots: 需要被添加的新slot
    return: none
    '''
    def addSlots(self, slots: set):
        for s in slots:
            self.slots.add(s)
            self.stage.rbtSlotsID[s].add(self.mId)

    '''
    description: 将self.slots和stage.rbtSlotsID中自己的信息进行删除
    param: slots: 需要被删除的slot
    return: none
    '''
    def delSlots(self, slots: set):
        for s in slots:
            self.slots.discard(s)
            self.stage.rbtSlotsID[s].discard(self.mId)

    '''
    description: 如果当前时间自己可以发送消息，就把自己发送缓存里的消息发送给自己的出邻居。
    param: nowSlot:int,当前时间
    return: sendLen:int，发送了多少个消息包
    '''
    def communicateTo(self, nowSlot):
        sendLen = 0
        if not (self.isCommunicating and (nowSlot in self.slots)):
            if self.status == 'online-pre' and nowSlot == 0:
                self.hasSendMsgFlag = 1

            return sendLen

        logging.info('\tid: {} send message: --> {}'.format(self.mId, [r.mId for r in self.outRobots if r.status != 'entering']))
        logging.info('\t\t{}'.format([(msg.sourceId, msg.relayId, msg.payload) for msg in self.sendMsgBuffer]))

        delLen = 0
        for robot in self.outRobots:
            if (self.mId, robot.mId) in self.stage.collRobots:
                delLen += 1

                logging.info('\tid: {}, cannot send to {}, because collision recoded in stage.collRobots[]'.format(self.mId, robot.mId))
                continue

            robot.recvMsgBuffer.extend(self.sendMsgBuffer)
            sendLen += len(self.sendMsgBuffer)

        # APP消息不计数
        for msg in self.sendMsgBuffer:
            if 'APP' in msg.payload:
                sendLen -= len(self.outRobots) + delLen

        self.sendMsgBuffer.clear()

        # 标志位置1，表明我已经把消息发送出去了。
        if self.status != 'online-pre':
            self.hasSendMsgFlag = 1

        return sendLen

    '''
    description: 1、发现360° Rs感知范围内的所有机器人对象；2、发现FOV 通信范围内的所有机器人对象
    param: none
    return: none
    '''
    def sense(self):
        self.robotsInRs.clear()
        # 这里光通信机器人通信范围为扇形，而感知是360度的，与师兄的实现刚好是相反的，那我们就反着用就好了
        obj_in_sense_length = getObjectInRange(self.mPos, self.mCommunicationRange)  # 感知距离以内的机器人
        obj_in_sense_length.remove(self)  # 排除自己

        if len(obj_in_sense_length) == 0:
            return

        self.robotsInRs = obj_in_sense_length

        self.outRobots.clear()

        obj_in_comm_length = getObjectInRange(self.mPos, self.mSenseDistance)  # 通信距离以内的机器人
        obj_in_comm_length.remove(self)  # 排除自己

        if len(obj_in_comm_length) == 0:
            return

        # 挑选出夹角以内的机器人，即xy平面内的偏航角度正负self.mSenseAngle以内，与xy平面的夹角正负self.mSenseAngle以内
        for robot in obj_in_comm_length:
            # xy平面内的偏航角度
            angle_in_xy = ComObject.getAngleBetweenXandVector(self.mPos, robot.mPos, plat='xy') - self.mDirection
            # 与xy平面的夹角
            angle_with_xy = ComObject.getAngleBetweenXandVector(robot.mPos, self.mPos, plat='o-xy')
            if angle_in_xy > math.pi:
                angle_in_xy = -2*math.pi + angle_in_xy
            if angle_in_xy < -math.pi:
                angle_in_xy = 2*math.pi + angle_in_xy
            if angle_with_xy > math.pi:
                angle_with_xy = -2*math.pi + angle_with_xy
            if angle_with_xy < -math.pi:
                angle_with_xy = 2*math.pi + angle_with_xy
            if (-self.mSenseAngle/2) <= angle_in_xy <= (self.mSenseAngle/2) and \
                    (-self.mSenseAngle/2) <= angle_with_xy <= (self.mSenseAngle/2):
                # 有新的出邻居了，置1，等待机器人online状态以后发生新的NEIGHBORS消息包,注意这里做了个弊，对于entering状态的机器人，不作数。
                if (robot.mId not in self.oldOutRobotsRecoder.keys() and robot.status != 'entering') or \
                        (robot.status == 'online-pre' and self.oldOutRobotsRecoder[robot.mId] == 'entering'):

                    # if self.mId != 4:
                    #     print('here4')

                    self.hasOutRobotChangedFlag = 1
                self.oldOutRobotsRecoder.pop(robot.mId, '404')

                self.outRobots.append(robot)

        # 原本的出邻居现在不是了，置1，等待机器人online状态以后发生新的NEIGHBORS消息包,注意这里做了个弊，对于entering状态的机器人，不作数。
        for id in self.oldOutRobotsRecoder.keys():
            r = list(self.stage.mRobotList.nodes)[id]
            if r.status != 'entering':
                self.hasOutRobotChangedFlag = 1

        # 记录此处感知结果。
        self.oldOutRobotsRecoder = {r.mId: r.status for r in self.outRobots}

    # region generate msg功能及相关函数
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------generate msg 功能及相关函数----------------------------------------------'
    '-------------------------------------------------START------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'

    '''
    description: 生成要发送的信息：1、转发信息; 2、根据出邻居生成请求信息
    param: genObj： dict = {‘type’: (str)type, 'info': (obj | list | set | dict ...)info}
                  'type': 'RELAY'--转发消息； 
    return: none
    '''
    def genVLCMsg(self, genObj):
        # APP消息包
        if genObj['type'] == 'APP':
            app = VLCMsg(self.mId, isBroadcasting=True)
            app.payload['APP'] = None
            self.sendMsgBuffer.append(app)

        # 中继转发消息包
        if genObj['type'] == 'RELAY':
            msg = copy.deepcopy(genObj['info'])
            if msg.isSetTransLimit:  # 如果消息设置了洪范寿命
                if msg.transNum == msg.transLimit:
                    return
                else:
                    msg.transNum += 1
            msg.relayId = self.mId
            self.sendMsgBuffer.append(msg)

        # REQ消息包
        if genObj['type'] == 'REQ':
            req = VLCMsg(self.mId, isBroadcasting=True)
            req.payload['REQ'] = copy.deepcopy(genObj['info'])
            self.sendMsgBuffer.append(req)

        # NEIGHBORS消息包
        if genObj['type'] == 'NEIGHBORS':
            nbr = VLCMsg(self.mId, isFlooding=True)
            nbr.seq = self.floodSeq
            nbr.relayId = self.mId
            self.floodSeq += 1
            nbr.payload['NEIGHBORS'] = copy.deepcopy(genObj['info'])
            self.sendMsgBuffer.append(nbr)

        # RES消息包
        if genObj['type'] == 'RES':
            res = VLCMsg(self.mId, isFlooding=True)
            res.seq = self.floodSeq
            res.relayId = self.mId
            self.floodSeq += 1
            for key, vals in genObj['info'].items():
                res.payload[key] = copy.deepcopy(vals)
            self.sendMsgBuffer.append(res)

        # 冲突包
        if genObj['type'] == 'COLLISION-ADD':
            coll = VLCMsg(self.mId, isFlooding=True)
            coll.seq = self.floodSeq
            coll.relayId = self.mId
            self.floodSeq += 1
            coll.payload['COLLISION-ADD'] = copy.deepcopy(genObj['info'])
            self.sendMsgBuffer.append(coll)

    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------generate msg 功能及相关函数----------------------------------------------'
    '-------------------------------------------------END------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    # endregion

    # region receive and process msg功能及相关函数
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------receive and process msg 功能及相关函数-----------------------------------------'
    '-------------------------------------------------START------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'

    '''
    description: 利用接收到的信息中的出入邻居信息更新我的graph
    param: id：发送该数据包的源机器人id
           neighbor0s：[[], []] 该机器人的入出邻居id
    return: none
    '''
    def updateGraph(self, id, neighbors):
        self.graph.add_nodes_from(neighbors[0])
        self.graph.add_nodes_from(neighbors[1])

        if id in self.graph.nodes:
            self.graph.remove_node(id)  # 删除原本图上的节点id以及与其相连接的所有边
        self.graph.add_node(id)  # 重新添加一个id节点

        # 防止生成回环的边（4，4）这种
        neighbors[0] = list(set(neighbors[0]))
        neighbors[1] = list(set(neighbors[1]))
        if id in neighbors[0]:
            neighbors[0].remove(id)
        if id in neighbors[1]:
            neighbors[1].remove(id)

        self.graph.add_edges_from([(r, id) for r in neighbors[0]])
        self.graph.add_edges_from([(id, r) for r in neighbors[1]])

    '''
    description: 消息解码：1、REQ消息：如果没有对齐做出过回应，则将请求相关的信息存储到self.resRefInfo中，方便后续为其申请不冲突的slot
                           2、slot消息包：直接将数据融合到self.proSlotInfo中
                           3、NEIGHBORS消息：[[in], [out]] 入邻居ids和出邻居ids， 将其同步更新到self.graph中。
                           4、APP消息：实际上是机器人之间传递的应用信息，但仿真中不设置应用信息，就pass
    param: msg: 接收到的VLCMsg消息包
    return: none
    '''
    def decoderVLCMsg(self, msg: VLCMsg):
        if 'APP' in msg.payload.keys():
            return

        if 'REQ' in msg.payload.keys():
            if self.status == 'online-pre':
                return
            sid = msg.sourceId
            if sid not in self.hasResIds:
                # print('id:', self.mId, 'received a request msg from id:', sid)
                logging.info('\tid: {}, receive a Request msg: {} from sourceId: {}'.format(self.mId, msg.payload['REQ'], msg.sourceId))
                self.resRefInfo[sid] = copy.deepcopy(msg.payload['REQ'])
            return

        if 'NEIGHBORS' in msg.payload.keys():
            # print('id:', self.mId, 'received a in neighbors msg from id:', payload['NEIGHBORS'][0])
            logging.info('\tid: {}, receive a Neighbors msg: {} from sourceId: {}, relayId: {}'.format(self.mId, msg.payload['NEIGHBORS'], msg.sourceId, msg.relayId))
            self.updateGraph(msg.sourceId, msg.payload['NEIGHBORS'])
            return

        if 'COLLISION-ADD' in msg.payload.keys():
            logging.info('\tid: {}, receive a Collision-ADD msg: {} from sourceId: {}, relayId: {}'.format(self.mId, msg.payload['COLLISION-ADD'], msg.sourceId, msg.relayId))
            self.addCollisions(msg.payload['COLLISION-ADD'])
            return

        # print('id:', self.mId, 'received a in slot msg:', payload)
        logging.info('\tid: {}, receive some Slot msg: {} from sourceId: {}, relayId: {}'.format(self.mId, msg.payload, msg.sourceId, msg.relayId))
        for id, val in msg.payload.items():
            if val['type'] == 'slot-add':
                if id not in self.proSlotInfo.keys():
                    self.proSlotInfo[id] = {'type': '', 'slots': set()}
                for s in val['slots']:
                    self.proSlotInfo[id]['slots'].add(s)
            elif val['type'] == 'slot-rdu':
                if id not in self.proSlotInfo.keys():
                    self.proSlotInfo[id] = {'type': '', 'slots': set()}
                for s in val['slots']:
                    self.proSlotInfo[id]['slots'].discard(s)
            elif val['type'] == 'slot-rpl':
                self.proSlotInfo[id] = copy.deepcopy(val)
            else:
                logging.info('\t!!!VLCMsgError:(fun: decoderVLCMsg) msg has no type, please check...!!!')
                print('\033[1;30;31m\tVLCMsgError:(fun: decoderVLCMsg) msg has no type, please check...\033[0m')

    '''
    description: 对接收缓存self.recvMsgBuffer中接收到的信息进行解析。
                1、处于entering状态的机器人不参与信息的解析，防止机器人本身还处在运动状态中，解析出来邻居信息没有用，但是实际中，其他机器人发送的应用信息是可以接收解析的。
                2、对于Broadcasting消息包，直接解析就行，无需转发之类的操作。
                3、对于Flooding消息包，需要注意避免广播风暴，对于接收过的消息就不再次转发，具体：
                    举例：floodHistory[msg.sourceId] = [2,3,6]
                    说明：1、数组中的最大值6表示已经接收过来自msg.sourceId的seq为6的数据包了，
                          2、数组中小于6的值2，3表示还没接收到来自msg.sourceId的seq为2和3的数据包，
                          3、所有小于6但又没出现在数组中值，这里的0，1，4，5表示已经接收过来自msg.sourceId的seq为6的数据包了，
                             这里隐去是为了节约存储空间。
                          4、所有大于6但又没出现在数组中值，这里的7，8，9....表示还没接收过来自msg.sourceId的seq为7，8，9....的数据包
                4、对于单播路由消息，进行具体解析就行。
    param: none
    return: recvIds: set  接收到信息的源ID数组，用于后续更新入邻居。
    '''
    def processRecvMsg(self):
        if self.status == 'entering':
            return []

        recvIds = set()
        for info in self.recvMsgBuffer:
            if info.isFlooding:  # 如果是洪泛数据包：解包，判断，转发
                # 如果洪泛包寿命已经超过极限，误传播，虽然理论上不可能，防止情况发生
                if info.isSetTransLimit and (info.transNum > info.transLimit):
                    recvIds.add(info.relayId)
                    continue

                # 如果是自己发送的数据包又回来了
                if info.sourceId == self.mId:
                    # print('id:', self.mId, 'receive a cycle msg:', msg.payload)
                    recvIds.add(info.relayId)
                    continue

                # 历史记录表示没有接收过来自该id的数据包
                if self.floodHistory.get(info.sourceId) is None:
                    self.floodHistory[info.sourceId] = list(range(info.seq + 1))  # 记录该id的数据包的seq值，以后再接收到来自该id的seq数据包就忽视

                    self.decoderVLCMsg(info)

                    recvIds.add(info.relayId)
                    self.genVLCMsg({'type': 'RELAY', 'info': info})  # 接收完成之后传到发送缓存中，准备进行转发
                    continue

                # 以下代码都是如果历史记录中有接收过来自该id的机器人的数据包，则进行进一步判断
                maxhist = max(self.floodHistory[info.sourceId])
                # 注释Flooding情况1
                if info.seq == maxhist:
                    # print('id:', self.mId, 'has received this msg: ', msg.payload, ' with case 1')
                    recvIds.add(info.relayId)
                    continue

                # 注释Flooding情况2
                if info.seq in self.floodHistory[info.sourceId]:  # 当前数据包seq值在floodHistory中
                    # print('id:', self.mId, 'receive a msg: ', msg.payload, ' with case 2')
                    self.floodHistory[info.sourceId].remove(info.seq)

                    self.decoderVLCMsg(info)

                    recvIds.add(info.relayId)
                    self.genVLCMsg({'type': 'RELAY', 'info': info})  # 接收完成之后传到发送缓存中，准备进行转发
                    continue

                # 注释Flooding情况3
                if info.seq < maxhist:
                    # print('id:', self.mId, 'has received this msg: ', msg.payload, ' with case 3')
                    recvIds.add(info.relayId)
                    continue

                # 注释情况4
                if info.seq > maxhist:
                    # print('id:', self.mId, 'receive a msg: ', msg.payload, ' with case 4')
                    self.floodHistory[info.sourceId].remove(maxhist)
                    self.floodHistory[info.sourceId].extend(list(range(maxhist + 1, info.seq + 1)))

                    self.decoderVLCMsg(info)

                    recvIds.add(info.relayId)
                    self.genVLCMsg({'type': 'RELAY', 'info': info})  # 接收完成之后传到发送缓存中，准备进行转发

            elif info.isBroadcasting:  # 不是洪泛包，但是是广播包，表示是传给邻居的包：解包，判断
                self.decoderVLCMsg(info)
                recvIds.add(info.sourceId)

            else:   # 非洪泛，非广播，定向数据包
                recvIds.add(info.sourceId)
                if info.targetId == self.mId:  # 传给我的定向数据包：解包。
                    pass
                elif self.mId == info.linkPath[0]:  # 只是经过我这里：顺着链路传递。
                    pass
                else:  # 无意中接收到的定向包，不经过我这里：抛弃。
                    pass

        return recvIds

    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------receive and process msg 功能及相关函数-----------------------------------------'
    '-------------------------------------------------END------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    # endregion

    # region toOnlinePre功能及相关函数
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------toOnlinePre功能及相关函数------------------------------------------------'
    '-------------------------------------------------START------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'

    '''
    description: 获取到一个闲置的目标位置，自动设置到自己的状态中。
    param: none
    return: none
    '''
    def acquireTargetPos(self):
        # 只为处在entering状态的机器人分配位置。机器人z轴位置大于400，说明正在赶往目标，不分配位置。
        if self.status != 'entering' or self.mPos[2] > 400:
            return
        self.mPosIndexInMap, self.mTarget = self.stage.updateMap()

        logging.info('\tid: {}-entering, acquire a target position: {} - {}'.format(self.mId, self.mPosIndexInMap, self.mTarget))

    '''
    description: 调用stage中的方法，计算我(自己)目前到网络中，有没有形成连通图？连通了就可以转为online-pre了。
    param: none
    return: 0: 机器人没有至少一个online入邻居和至少一个online出邻居，没有形成连通
            1：有了，形成连通
    '''
    def calcSelfConnected(self):
        ret = self.stage.calcRobotIn(self.mId)
        flag = 0
        for robot in ret:
            flag = 1 if robot.status == 'online' else 0
        if not flag:
            return 0

        flag = 0
        for robot in self.outRobots:
            flag = 1 if robot.status == 'online' else 0
        if not flag:
            return 0

        return 1

    '''
    description: 处于erntering状态的机器人的任务：
                 1、不处于entering的机器人不执行
                 2、为刚出生的机器人分配地图上的一个目标位置，让机器人赶过去
                 3、到达目标位置之后，判断连通性，不连通就原地转圈圈，转了一圈还不连通的话就随机回一个出生位置，等待新的目标位置。
    param: none
    return: none
    '''
    def toOnlinePre(self):
        if self.status != 'entering':
            return

        # 处于出生状态
        if self.mPos[2] <= 400:
            self.acquireTargetPos()

        # 到达目标位置了停住了
        elif self.isStopping() and self.mPos[2] == 500:
            if self.calcSelfConnected():
                self.status = 'online-pre'
                print('\033[1;32m\t id: ', self.mId, 'change status to online-pre\033[0m')
                logging.info('\tid: {} change status to online-pre'.format(self.mId))

            else:
                angle = -math.pi if (self.mDirection + 0.54) > math.pi else (self.mDirection + 0.54)
                if 0 < (self.initDirection - self.mDirection) <= 0.54:
                    if self.mPosIndexInMap[0] != -1:
                        print('???????????????????????? 发现一个无用的位置', self.mPosIndexInMap[0], self.mPosIndexInMap[1], '????????????????????????????????')
                        logging.info('\tid: {}, find a useless position in stage.map: {}'.format(self.mId, self.mPosIndexInMap))

                        self.stage.setMapUseless(self.mPosIndexInMap[0], self.mPosIndexInMap[1])
                    else:
                        print('???????????????????????? 随机接入了一个无用的位置，重新随机接入??????????????????????????')
                        logging.info('\tid: {}, find a useless random chosen')

                    x = random.uniform(-300, 300)
                    y = random.uniform(-300, 300)
                    self.mTarget = np.array([x, y, 400], dtype=np.float32)
                else:
                    self.setDirection(angle)
        # 在赶往目标的的过程中
        else:
            self.initDirection = self.mDirection

    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------toOnlinePre功能及相关函数------------------------------------------------'
    '-------------------------------------------------EDN--------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    # endregion

    # region toOnline功能及相关函数
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------toOnline 功能及相关函数--------------------------------------------------'
    '-------------------------------------------------START------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'

    '''
    description: 将inRobotsIdsList数组对应机器人对象添加进inRobots
    param: idsList: 接收到信息的机器人id集合
    return: none
    '''
    def addInRobots(self, idsList: list):
        for id in idsList:
            robot = list(self.stage.mRobotList.nodes)[id]
            if robot in self.inRobots:
                continue
            self.inRobots.append(robot)
            # 有新的入邻居了，置1，等待机器人online状态以后发生新的NEIGHBORS消息包
            self.hasInRobotChangedFlag = 1

    '''
    description: 将inRobotsIdsList数组对应机器人对象从inRobots删掉
    param: idsList: 需要删除的机器人id集合
    return: none
    '''
    def delInRobots(self, idsList: list):
        for id in idsList:
            robot = list(self.stage.mRobotList.nodes)[id]
            if robot not in self.inRobots:
                continue
            self.inRobots.remove(robot)
            # 有机器人不再是自己的入邻居了，置1，等待机器人online状态以后发生新的NEIGHBORS消息包
            self.hasInRobotChangedFlag = 1

    '''
    description:enrtering 状态的机器人不更新自己的入机器人，保持自己inRobotsIdsRecoder长度不变。
                每个slot更新过程中，从哪些id的机器人那里接收到信息，就将这些id对应的机器人添加到self.inRobots[]中。 
                同时对于inRobotsIdsRecoder最前面一个数组集合中的机器人id，如果后面的数组中没有出现过表示这一帧数据中都没有接收到该机器人的数据，表示断开了。
    param: inRobotsIdsList: 接收到信息的机器人id集合
    return: none
    '''
    def updateInRobots(self, inRobotsIdsList: list):
        if self.status == 'entering':
            return

        self.addInRobots(inRobotsIdsList)

        if self.stage.nowTime in range(self.stage.formerSlots + 1, self.stage.formerSlots + self.stage.preSlots):
            # 为了防止机器人在预备slot的时间下从entering转变为online-pre，这时候inRobotsIdsRecoder还是空的。
            if len(self.inRobotsIdsRecoder) == 0:
                self.inRobotsIdsRecoder.append([])

            self.inRobotsIdsRecoder[-1].extend(list(set(inRobotsIdsList)))
            self.inRobotsIdsRecoder[-1] = list(set(self.inRobotsIdsRecoder[-1]))

        elif len(self.inRobotsIdsRecoder) >= (self.stage.formerSlots + 1):
            self.inRobotsIdsRecoder.append(copy.deepcopy(list(set(inRobotsIdsList))))

            popList: list = self.inRobotsIdsRecoder.pop(0)
            recoderList = []
            for c in self.inRobotsIdsRecoder:
                recoderList.extend(c)
            for id in popList:
                if id in recoderList:
                    popList.remove(id)

            self.delInRobots(popList)

        else:
            self.inRobotsIdsRecoder.append(copy.deepcopy(list(set(inRobotsIdsList))))

    '''
    description: 判断机器人是否达到转变为online的条件---如果self.slots不为空（说明有预备slot）且 self.proSlotInfo[self.mId]['slots']不为空（说明接收到关于自己的slot信息了）
    param: none
    return: 0 or 1, 0: 机器人还没有达到转变为online的条件， 1：已经达到条件了
    '''
    def judgeHasOnline(self):
        if len(self.slots) > 1:
            print('\033[1;30;31m\ttoOnline()函数逻辑错误，judgeHasOnline()函数中self.slot中不应该有多个元素\033[0m')
            return 0

        if len(self.slots) == 1 and list(self.slots)[0] not in range(self.stage.formerSlots, self.stage.formerSlots + self.stage.preSlots):
            print('\033[1;30;31m\ttoOnline()函数逻辑错误，judgeHasOnline()函数中self.slot中的元素应该在预备slot选项中\033[0m')
            return 0

        if self.slots and self.proSlotInfo[self.mId]['slots']:
            return 1
        return 0

    '''
    description: 将发送缓存中还未发送出去的REQ Msg给删除掉，不用发了。
    param: none
    return: none
    '''
    def delReqNotSend(self):
        delList = []
        for msg in self.sendMsgBuffer:
            if 'REQ' in msg.payload:
                delList.append(msg)

        for msg in delList:
            self.sendMsgBuffer.remove(msg)

    '''
    description: 对于处在online-pre的机器人，其占用一个pre slot发送请求信息，就可能会出现多个机器人用一个pre slot但是他们之间会产生冲突。所以变着slot发。
    param: none
    return: none
    '''
    def allocNewPreSlot(self):
        if len(self.slots) > 1:
            print('\033[1;30;31m\ttoOnline()函数逻辑错误，judgeHasOnline()函数中self.slot中不应该有多个元素\033[0m')
            return

        if len(self.slots) == 1 and list(self.slots)[0] not in range(self.stage.formerSlots, self.stage.formerSlots + self.stage.preSlots):
            print('\033[1;30;31m\ttoOnline()函数逻辑错误，judgeHasOnline()函数中self.slot中的元素应该在预备slot选项中\033[0m')
            return

        self.changeSlots({random.randint(1, self.stage.preSlots) + self.stage.formerSlots - 1})

    '''
    description:每个机器人在刚变成online-pre时，需要静静等候一个frame的时间，
                1、一方面是为了聆听接收到的数据包，更新自己的入邻居；
                2、另一方面是为了从入邻居那里获得关于网络的部分信息；
                3、时间同步（这里仿真不仿了，但实际是这样一个逻辑。）
    param: none
    return: 0 or 1， 0： 一帧还没过去；  1：一帧已经结束。
    '''
    def waitFrame(self):
        # 通过self.inRobotsIdsRecoder来判断是否过去了一帧
        if len(self.inRobotsIdsRecoder) < (self.stage.formerSlots + 1):
            return 0

        logging.info('\tid: {}, has waited a frame with inRobots: {}, now i get info from this inRobots:'.format(self.mId, [r.mId for  r in self.inRobots]))

        # 在不分簇的协议中，每个机器人需要知道所有其他机器人的信息，所以直接融合所有入邻居的信息作为自己的初始信息。
        for robot in self.inRobots:
            # slots信息
            for key, vals in robot.proSlotInfo.items():
                if key in self.proSlotInfo:
                    self.proSlotInfo[key]['slots'] = self.proSlotInfo[key]['slots'] | vals['slots']
                else:
                    self.proSlotInfo[key] = copy.deepcopy(vals)

            # graph信息
            self.updateGraph(self.mId, [[r.mId for r in self.inRobots], [r.mId for r in self.outRobots if r.status != 'entering']])
            newNodes = set(self.graph.nodes) | set(robot.graph.nodes)
            newEdges = set(self.graph.edges) | set(robot.graph.edges)
            self.graph.remove_nodes_from(copy.deepcopy(self.graph.nodes))
            self.graph.add_nodes_from(newNodes)
            self.graph.add_edges_from(newEdges)

            # collision信息
            self.addCollisions(robot.collisions)

        logging.info('\t\tslots info: {}'.format(self.proSlotInfo))
        logging.info('\t\tgraph info: nodes: {}, edges: {}'.format(self.graph.nodes, self.graph.edges))
        logging.info('\t\tcolls info: {}'.format(self.collisions))

        # 如果在分簇的协议中，就需要有选择的对信息进行融合。

        return 1

    '''
    description: 1、只online-pre状态的机器人执行
                 2、等待1frame，更新入邻居，获得关于网络的初始化信息。
                 3、判断状态转变，到达就转为online
                 4、重新为下次发送请求申请一个新的pre slot
                 5、生成REQ Msg
    param: none
    return: none
    '''
    def toOnline(self):
        if self.status != 'online-pre':
            return

        # wait a frame
        if not self.hasGetInitInfoFlag:
            self.hasGetInitInfoFlag = self.waitFrame()
        if not self.hasGetInitInfoFlag:
            return

        # 等待一个frame了，正式接入。改变标志位
        self.setStartEndAccessFlag(start=1, end=0)

        # 如果达到online转变条件了
        if self.judgeHasOnline():
            # 删除发送buffer中可能存在的还没发送的request请求信息。
            self.delReqNotSend()

            # 更新自己接收的关于自己的slot信息到self.slots中。
            self.changeSlots(self.proSlotInfo[self.mId]['slots'])

            self.status = 'online'

            # 接入成功了，改变标志位。
            self.setStartEndAccessFlag(start=0, end=1)

            print('\033[1;32m\t id: ', self.mId, 'change status to online\033[0m')
            logging.info('\tid: {} change status to online, access in successfully'.format(self.mId))
            return

        # 如果之前的REQ信息已经发送出去了
        if self.hasSendMsgFlag:
            # 新生成REQ Msg
            self.genVLCMsg({'type': 'REQ', 'info': [r.mId for r in self.outRobots if r.status != 'entering']})

            # 重新为自己分配一个pre slot
            self.allocNewPreSlot()

            self.hasSendMsgFlag = 0

            # print('id:', self.mId, ' status: pre slot:', self.slots,
            #       'gen request msg---with out:', [robot.mId for robot in self.outRobots],
            #       ' out-online:', [robot.mId for robot in self.outRobots if robot.status == 'online'],
            #       ' out-online-pre:', [robot.mId for robot in self.outRobots if robot.status == 'online-pre'])
            logging.info('\tid: {}-online-pre, next send slot: {} gen Request msg with outRobots: online-{}, online-pre-{}'
                         .format(self.mId, self.slots,
                                 [robot.mId for robot in self.outRobots if robot.status == 'online'],
                                 [robot.mId for robot in self.outRobots if robot.status == 'online-pre']
                                 ))

    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------toOnline功能及相关函数---------------------------------------------------'
    '-------------------------------------------------END------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    # endregion

    # region onlineTask功能及相关函数
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------onlineTask 功能及相关函数------------------------------------------------'
    '-------------------------------------------------START------------------------------------------------------------'
    '******************************************************************************************************************'
    '******************************************************************************************************************'

    '''
    description: 遍历self.collisions中的项，如果发现有些已经确实没必要保持了，就删除，并发送出去。
    param: 
    return: 
    '''
    def delCollisions(self):
        pass

    '''
    description: 判断检测到的接入冲突是否自己已经知道了，self.collisions中是否已经有了，有了就忽略，没有就更新，并发送出去让其他人知道
    param: colls: 接入冲突
    return: remains: 与本地的collisions信息进行对比，除去那些已经知道的、将剩下的返回，这些是真正对自己的collisions做出了更新的。
    '''
    def addCollisions(self, colls: list):
        remains = []
        for coll in colls:
            if coll in self.collisions:
                continue

            if len(self.collisions) == 0:
                self.collisions.append(coll)
                remains.append(coll)
                continue

            removes = []
            for existColl in self.collisions:
                if coll.issubset(existColl):
                    break
                if existColl.issubset(coll):
                    self.collisions.append(coll)
                    removes.append(existColl)
                    if existColl in remains:
                        remains.remove(existColl)
                    remains.append(coll)
                    break
                if self.collisions.index(existColl) == len(self.collisions) - 1 and\
                        not coll.issubset(existColl) and\
                        not existColl.issubset(coll):
                    self.collisions.append(coll)
                    remains.append(coll)

            for coll in removes:
                if coll not in self.collisions:
                    continue
                self.collisions.remove(coll)

        return remains

    '''
    description: 将最新检测到的接入冲突发送出去。
    param: colls: 最近检测到的接入冲突。
    return: none
    '''
    def accessingCollsSend(self, colls: list):
        r = self.addCollisions(colls)

        if len(r) > 0:
            self.genVLCMsg({'type': 'COLLISION-ADD', 'info': r})

            logging.info('\t\tAccess in Collisions: {}, with all collisions i knew: {}'.format(colls, self.collisions))

    '''
    description: 根据自身的入邻居和掌握的slot信息，判断是否有融入冲突发生，有就发送出去。
    param: none
    return: none
    '''
    def mergingCollsSend(self):
        mcolls = []
        for robot in self.inRobots:
            outIds = [item[1] for item in self.graph.out_edges(robot.mId)]  # outIds数组表示机器人robot的出邻居id数组,
            for rid in outIds:
                ridInIds = [item[0] for item in self.graph.in_edges(rid)]  # ridInIds数组表示id=rid机器人的入邻居id数组,,
                if robot.mId in ridInIds:
                    ridInIds = np.setdiff1d(ridInIds, [robot.mId])  # 注意要剔除自己的id

                for v in ridInIds:  # 那么每个id=v的机器人应该与id=robot.mId的机器人没有slot交集
                    if v in self.proSlotInfo:
                        s = robot.slots & self.proSlotInfo[v]['slots']

                        if s:
                            mcolls.append({robot.mId, v})
                            # print(self.mId, '-merging collision detected: ',
                            #       {robot.mId, v}, 'with common out robot id:', rid)

                    else:  # 如果id=v的机器人不在自己的p_acsInfo中记录，表示id=v的机器人slot信息还没有同步到我这里来，就是我忽略一个机器人的信息。
                        mcolls.append({robot.mId, v})

                        # print('id:', self.mId, ' because i do not know the slot about robot:', v,
                        #       'so i ignore the difference among:', {robot.mId, v},
                        #       'in merging collision detecting function', 'with common out robot id:', rid)

        r = self.addCollisions(mcolls)

        if len(r) > 0:
            self.genVLCMsg({'type': 'COLLISION-ADD', 'info': r})

            logging.info('\t\tMerging Collisions: {}, with all collisions i knew: {}'.format(r, self.collisions))
            # print('id:', self.mId, 'collisions have collected: ', self.collisions, 'with new merging collisions may happened: ', r)

    '''
    description: 每个机器人根据自身的self.collisions来判断自身是否需要暂时放弃一些slot来避免冲突的发生，方法就是：
                 遍历collisions数组，每一个coll:set里面存储的是机器人id，表示这里面的几个机器人之间应该slot没有重复。
                 对其进行判断，如果有重复，就找出哪些机器人之间存在重复,找出这些机器人中谁的slot最少，其他的机器人都应该暂时放弃这个slot以避免冲突。
    param: none
    return: ret: set, 记录在这个过程中我自身需不需要暂时放弃一些slot，返回，以便通信后加回来。
    '''
    def collsSolve(self):
        ret = set()
        for coll in self.collisions:  # arr就是一个互斥数组
            if len(coll) < 2:
                # print('\033[1;30;31m ERROR:(fun: collsSolve) collision set len must >= 2\033[0m')
                logging.info('\tERROR(id: %d):collision set len must >= 2, with this collision set: {}'.format(coll) % self.mId)
                continue

            muxDict = {mId: copy.deepcopy(self.proSlotInfo[mId]['slots']) for mId in coll if mId in self.proSlotInfo}

            if len(muxDict) < 2:
                # print('\033[1;30;33m\tWARRING:(fun: collsSolve) lack of collision robot information, '
                #       'id:{} i cannot solve this collision {} \033[0m'.format(self.mId, coll))
                logging.info('\tWARRING(id: %d):(fun: collsSolve) '
                             'lack of information about collision robotIds: {},'
                             ' info only know: {}'.format(coll, muxDict) % self.mId)
                continue

            # 删除前面为了避免冲突已经删除掉的机器人的slot，防止把机器人的slot都删除没了，就不能发送信息了。
            for item in ret:
                if item[0] in muxDict:
                    muxDict[item[0]].discard(item[1])

            s1 = set(range(self.stage.formerSlots + self.stage.preSlots))  # s1到最后就是coll互斥数组机器人之间的slot有没有存在冲突，有就不为空，为空就说明没有冲突
            for vals in muxDict.values():
                s1 = s1 & vals

            for s in list(s1):
                muxDict2 = [(mId, len(vals)) for mId, vals in muxDict.items() if s in vals]
                mIds = [item[0] for item in muxDict2]
                lens = [item[1] for item in muxDict2]

                if self.mId in mIds:
                    logging.info('\tid: {}, collIds: {}, colls slots: {}, know info: {}'.format(self.mId, coll, s1, muxDict))

                    templ = [item for item in lens if item > 1]
                    if len(templ) < 1:
                        logging.info('\tcause each robot has only one slot, do not delete slot.')
                        continue

                minLenId = mIds[lens.index(min(lens))]

                if self.mId in mIds and minLenId != self.mId:  # 如果是自己需要删除某个slot，就记录，返回，如果不是自己就pass
                    if len(self.slots) == 1:
                        logging.info('\tcause i({}) has only one slot, do not delete my slot.'.format(self.mId))
                        continue

                    self.delSlots({s})
                    ret.add((self.mId, s))

                    # print('id: ', self.mId, 'delete slot:', s, 'to avoid collisions:', coll, s1)
                    logging.info('\tid: %d' % self.mId + ' delete slot: %d' % s + ' to avoid collisions: {}'.format(coll))

        return ret

    '''
    description: 往self.hasResIds添加id，表示已经对该机器人做出response了，不再重复response了
    param: newId: 新加进来的id：int
    return: none
    '''
    def addHasResIds(self, newId):
        self.hasResIds.add(newId)

    '''
    description: 如果有id原本在self.hasResIds中，但是在self.proSlotInfo却没有该机器人的记录或者self.proSlotInfo[id]['slots']为空了，则从hasResIds中删除该id，后续能继续接收该机器人的REQ请求
    param: none
    return: none
    '''
    def delHasResIds(self):
        for id in self.hasResIds.copy():
            if id not in self.proSlotInfo.keys():
                self.hasResIds.discard(id)

                logging.info('\tid: {}: robot {} slots info not in my proSlotInfo, so i remove it from self.hasResIds'.format(self.mId, id))

            if len(self.proSlotInfo[id]['slots']) == 0:
                self.hasResIds.discard(id)

                logging.info('\tid: {}: robot {} slots info is empty in my proSlotInfo, so i remove it from self.hasResIds'.format(self.mId, id))

    '''
    description: 根据自身对网络的了解：连接关系以及各个机器人的slot信息，位提出申请的机器人id分配一个尽肯能不发生冲突的slot选择。
                选取规则：设新接入机器人为x，其出邻居们记为oi = [o1, o2, ...], 针对每个oi，其入邻居为sij = [si1, si2,...],则x的slot应该与每个sij的slot都不同。
                          因此我们先根据自己对于其他机器人的了解，收集尽可能多的sij所占用的slot集合N，那么x的slot选择就不能在N中，如果N == {1,2,...25}，
                          就统计在所有sij中，只占有一个slot的机器人的slot信息组成数组P，再统计占有复数个slot的机器人的slot信息组成数组Q，在Q中将P去除得到新的P，
                          在新的P中计算哪一个值出现的次数最少，就选择哪个作为最后选择。
    param: id: 提出申请的机器人id
           outIds: 提出申请的机器人的出邻居id（这其中不包括处在entering状态的机器人）
    return: s: set , 抉择出来的slot值。
            info: {}, 需要释放slot的机器人相关信息，需要发送出去的。
            colls: [], 由于对某些机器人的slot信息掌握不全面，忽视了某些可能存在的冲突，记录下来。
    '''
    def chooseSlot(self, id, outIds: list):
        colls = []

        csr = set(range(self.stage.formerSlots))
        temp = {}
        cannotc = set()  # 有些机器人只有一个slot，则这些slot不能被选择，如果需要剥夺其他机器人的slot的情况。

        for rid in outIds:  # 每个rid表示能被id=key的机器人的出邻居机器人id。
            if rid not in self.graph.nodes:
                nr = list(self.stage.mRobotList)[rid]
                logging.info('\t\toutRobot id {} not in my graph, so ignore this robot s inRobots with request robot'.format(rid))
                # print('id: ', self.mId, 'does not know the robot:{', rid, ', ', nr.status, '}', 'so ignore this robot s inRobots with request robot:', id)
                continue

            ridInIds = [item[0] for item in self.graph.in_edges(rid)]  # ridInIds数组表示id=rid机器人的入邻居id数组
            if id in ridInIds:
                ridInIds = np.setdiff1d(ridInIds, [id])  # 注意要剔除自己的id

            for v in ridInIds:  # 那么每个id=v的机器人应该与id=id的机器人没有slot交集
                if v in self.proSlotInfo.keys():
                    csr -= self.proSlotInfo[v]['slots']

                    if len(self.proSlotInfo[v]['slots']) > 1:
                        temp[v] = copy.deepcopy(self.proSlotInfo[v]['slots'])

                    if len(self.proSlotInfo[v]['slots']) == 1:
                        cannotc.add(list(self.proSlotInfo[v]['slots'])[0])

                else:  # 如果id=v的机器人的slot信息不在自己的proSlotInfo中记录，表示id=v的机器人slot信息还没有同步到我这里来，那么可能我后面选择的slot就会与其发生冲突。
                    # print('id:', self.mId, ' because i do not know the slot about robot:', v, 'so i ignore the difference among:', {id, v}, 'in func chooseSlot()')
                    colls.append({id, v})

        s = set()
        if len(csr) > 0:
            s.add(random.choice(list(csr)))

            logging.info('\t\tget a idle slot: {}'.format(s))

            return s, {}, colls

        # 到这里说明csr已经为空了，表明我们需要剥夺已经被占用的slot然后分配给机器人提出申请的机器人
        outIds = sorted(outIds)
        slen = round(len(outIds) / 2) + 1

        # 只有部分机器人会在这种情况下发res包
        if self.mId not in outIds[: slen]:
            logging.info('\t\ti have not the permission to response a slot.')
            return s, {}, colls

        # 计算每个slot被机器人占用的次数
        fCounter = {}
        for item in temp.values():
            for i in item:
                if i in cannotc:
                    continue
                elif i in fCounter.keys():
                    fCounter[i] += 1
                else:
                    fCounter[i] = 1

        list1 = list(fCounter.values())
        list2 = list(fCounter.keys())
        if len(list1) == 0:
            logging.info('\t\teach robot hasa only one slot, i cannot choose a slot to response.')
            return s, {}, colls

        # 找出次数最少的作为我们的选择目标
        ts = list2[list1.index(min(list1))]
        s.add(ts)
        logging.info('\t\tget a not idle slot: {}, may raise some slot-rdu packets'.format(s))

        # 找到哪些机器人占用了我们选择的slot，需要通知他们释放掉
        info = {}
        for key in temp.keys():
            if ts in temp[key]:
                info[key] = {'type': 'slot-rdu', 'slots': {ts}}

        print('@@@@@@@@@@@@@@@@ reduce robot:', info.keys(), ' with slot: ', ts, ' to guarantee new robot in @@@@@@@@@')
        return s, info, colls

    '''
    description: 先讲发送缓存中还未发送出去的我生成的NEIGHBORS消息删除，再根据传进来的入出邻居数组生成新的消息。
    param: 最近的入出邻居数组[[], []]
    return: none
    '''
    def genNewNeighborMsg(self, nbr):
        delList = []
        for msg in self.sendMsgBuffer:
            if msg.sourceId == self.mId and 'NEIGHBORS' in msg.payload.keys():
                delList.append(msg)
        for msg in delList:
            self.sendMsgBuffer.remove(msg)

        self.genVLCMsg({'type': 'NEIGHBORS', 'info': nbr})

    '''
    description: 1、只有online状态的机器人执行
                 2、对收到的请求进行response，其中需要基于自身所了解的网络信息，为机器人分配一个slot
                 3、当自身检测到入出邻居发生变化，发送新的NEIGHBORS消息包，并更新自己的graph 。注：出邻居变化指出邻居online或者online-pre状态的机器人发生变化。
                 4、如果自身的slot因为分布式被剥夺的原因导致到最后我没有slot可以发送信息了，改变状态
                 5、检测接入两种类型的接入冲突，发送出去。
                 6、每个发送slot都要确保发送一个APP消息包。
    param: none
    return: none
    '''
    def onlineTask(self):
        if self.status != 'online':
            return

        # 对比当前自身的slots和proSlotInfo[self.mId]之间的数据差别，保持更新。
        if self.slots != self.proSlotInfo[self.mId]['slots']:
            logging.info('\tid: {}, change self.slot: {} to {}'.format(self.mId, self.slots, self.proSlotInfo[self.mId]['slots']))

            self.changeSlots(self.proSlotInfo[self.mId]['slots'])

        # 如果自身的slot因为分布式被剥夺的原因导致到最后我没有slot可以发送信息了，改变状态
        if len(self.slots) == 0:
            self.status = 'online-pre'
            logging.info('\tid: {} change status to online-pre, cause all my slots been deleted')

        # 检测是否有机器人因为被剥夺了所有slot转变为online-pre状态，我需要从hasResIds中删除。以遍重新接收其请求。
        self.delHasResIds()

        # 发送新的NEIGHBORS消息包
        if self.hasOutRobotChangedFlag or self.hasInRobotChangedFlag:
            nbr = [[r.mId for r in self.inRobots], [r.mId for r in self.outRobots if r.status != 'entering']]

            self.genNewNeighborMsg(nbr)

            logging.info('\tid: {} gen Neighbors msg: {}'.format(self.mId, nbr))

            # 邻居发生了变化，更新一下graph
            self.updateGraph(self.mId, nbr)

            self.hasOutRobotChangedFlag = 0
            self.hasInRobotChangedFlag = 0

        # 发送RES消息包
        if len(self.resRefInfo) > 0:
            newColls = []
            for key, ids in self.resRefInfo.items():
                logging.info('\tid: {}: gen Response for robotID: {}: '.format(self.mId, key))

                s, delInfo, newColls = self.chooseSlot(key, ids)

                if s:
                    # 生成RES消息
                    self.genVLCMsg({'type': 'RES', 'info': {key: {'type': 'slot-add', 'slots': s}}})
                    self.addHasResIds(key)

                    # print('id:', self.mId, ' status: online, gen response msg for:', key, 'with slot:', s)

                if delInfo:
                    self.genVLCMsg({'type': 'RES', 'info': delInfo})

                    # 同步更新到自己的proSlotInfo中
                    for delId in delInfo.keys():
                        self.proSlotInfo[delId]['slots'] -= s

                    logging.info('\t\t\tslot-rdu packets: {}'.format(delInfo))

                # 同步更新到自己的proSlotInfo中
                if key not in self.proSlotInfo.keys():
                    self.proSlotInfo[key] = {'type': '', 'slots': s}
                else:
                    self.proSlotInfo[key]['slots'] = self.proSlotInfo[key]['slots'] | s

            if len(self.resRefInfo.keys()) > 1:  # 如果要response列表大于1，说明有多个机器人同时照到我了，二者要槽信息不同
                newColls.append(set(self.resRefInfo.keys()))
                # print('id: ', self.mId, 'response multi request a time, which may cause new collision: ', set(self.resRefInfo.keys()))

            # 检测是否发生了接入冲突，发送冲突数据包
            self.accessingCollsSend(newColls)

            self.resRefInfo.clear()

        # 融入冲突检测
        self.mergingCollsSend()

        # 冲突过期检测
        self.delCollisions()

        # 发送APP消息包
        if self.hasSendMsgFlag:
            self.genVLCMsg({'type': 'APP', 'info': None})
            self.hasSendMsgFlag = 0

    '******************************************************************************************************************'
    '******************************************************************************************************************'
    '-----------------------------------------onlineTask 功能及相关函数------------------------------------------------'
    '-------------------------------------------------END----------------------------------------------------------  --'
    '******************************************************************************************************************'
    '******************************************************************************************************************'
    # endregion

    '''
    description: 对接收到的信息进行处理的函数和更新自己的入邻居的函数的打包函数。在self.update()函数执行前执行。
    param: none
    return: none
    '''
    def updatePre(self):
        ret = self.processRecvMsg()
        self.recvMsgBuffer.clear()
        self.updateInRobots(ret)

    def update(self):
        # 计数接入时间
        if self.hasStartAccessFlag:
            self.accessCount += 1
            if self.accessCount >= 200:
                print('\033[1;31m\tid: {}, i cannot access in here, so i change a target position\033[0m'.format(self.mId))
                logging.info('\tid: {} cannot access in here cause i wasted a long time, so i change a target position')

                self.accessCount = 0
                self.setStartEndAccessFlag(start=0, end=0)
                self.status = 'entering'
                x = random.uniform(-300, 300)
                y = random.uniform(-300, 300)
                self.mTarget = np.array([x, y, 400], dtype=np.float32)

        self.toOnlinePre()

        self.toOnline()

        self.onlineTask()

        self.move()

        if self.hasEndAccessFlag:
            self.stage.accessCountHistory.append((self.mId, self.accessCount))
            self.accessCount = 0
            self.hasEndAccessFlag = 0
            self.stage.acesSuccessFlag -= 1
            self.stage.calcMetricsFlag = 1

    # def test(self):
    #     if self.mId == 0 and self.slots != {0,1}:
    #         print('0, error')
    #     if self.mId == 1 and self.slots != {2,3}:
    #         print('0, error')
    #     if self.mId == 2 and self.slots != {4,5}:
    #         print('0, error')
    #     if self.mId == 3 and self.slots != {6,7}:
    #         print('0, error')


# waitFrame中：如果在分簇的协议中，就需要有选择的对信息进行融合。
# processRecvMsg中：对于洪泛消息的接收解析是不是需要改一下范围啥的？ 针对单播路由消息的解析。
# updategraph中： 根据出入邻居更新图，是不是需要限定范围了？
# genVLCMsg中，生成一些洪泛包的时候，是不是需要考虑限定接收范围或者路径优化问题了，而不是无脑洪泛。
# accessingCollsDect和mergingCollsSend中，冲突消息包发送给其他所有机器人么？可不可以限制以下。
