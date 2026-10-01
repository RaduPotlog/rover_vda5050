#!/usr/bin/env python3
# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Minimal VDA 5050 2.0 master control, for exercising the rover's connector by hand.

    fake_master.py watch                       # print connection/state as they arrive
    fake_master.py order 2,0 4,0 4,2           # order from the current position via these points
    fake_master.py order --order-id o1 --update 1 --from-node n2 6,2   # stitch onto order o1
    fake_master.py pause | resume | cancel     # startPause / stopPause / cancelOrder
    fake_master.py aux on 1 3 | aux off 3      # enableAuxOutput / disableAuxOutput, outputs 1..6
    fake_master.py order 2,0 4,0 --node-action 1:enableAuxOutput:2:HARD \
                                 --node-action 2:disableAuxOutput:2    # aux actions on nodes
    fake_master.py factsheet                   # factsheetRequest, prints the answer
    fake_master.py drive-mode MANUAL           # setDriveMode (custom): MANUAL or AUTOMATIC

An order's first node is the rover's current position (read from its state), as VDA 5050
requires; each x,y after it is a released node joined by a released edge.
"""

import argparse
import itertools
import json
import sys
import threading
import time
import uuid
from datetime import datetime, timezone

from paho.mqtt import client as mqtt

VERSION = '2.0.0'


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


class Master:
    """One MQTT session speaking to one rover."""

    def __init__(self, args):
        self._args = args
        self._prefix = f'{args.interface}/v2/{args.manufacturer}/{args.serial}'
        self._header_ids = {}
        self._state = None
        self._state_event = threading.Event()
        self._messages = []
        self._lock = threading.Lock()

        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._client.on_message = self._on_message
        self._client.connect(args.host, args.port)
        for topic in ('state', 'connection', 'factsheet'):
            self._client.subscribe(f'{self._prefix}/{topic}', qos=1)
        self._client.loop_start()

    def close(self):
        self._client.loop_stop()
        self._client.disconnect()

    def _on_message(self, client, userdata, msg):
        payload = json.loads(msg.payload)
        topic = msg.topic.rsplit('/', 1)[-1]
        with self._lock:
            self._messages.append((topic, payload))
            if topic == 'state':
                self._state = payload
                self._state_event.set()

    def take_messages(self):
        with self._lock:
            messages, self._messages = self._messages, []
        return messages

    def state(self, timeout: float = 10.0):
        if not self._state_event.wait(timeout):
            sys.exit(f'No state from {self._prefix} within {timeout:.0f} s: is the connector up?')
        with self._lock:
            return self._state

    def publish(self, topic: str, body: dict):
        header_id = self._header_ids.get(topic, 0) + 1
        self._header_ids[topic] = header_id
        message = {
            'headerId': header_id,
            'timestamp': timestamp(),
            'version': VERSION,
            'manufacturer': self._args.manufacturer,
            'serialNumber': self._args.serial,
            **body,
        }
        self._client.publish(f'{self._prefix}/{topic}', json.dumps(message), qos=0).wait_for_publish()
        print(f'-> {topic}: {json.dumps(message)}')

    def instant_action(self, action_type: str, parameters: list = None):
        self.publish('instantActions', {'actions': [
            action(action_type, 'HARD', parameters or [])]})


def action(action_type: str, blocking_type: str, parameters: list) -> dict:
    return {
        'actionType': action_type,
        'actionId': str(uuid.uuid4()),
        'blockingType': blocking_type,
        'actionParameters': parameters,
    }


def outputs_parameter(outputs: list) -> list:
    return [{'key': 'outputs', 'value': outputs}]


def parse_node_action(text: str):
    """INDEX:TYPE:OUTPUTS[:BLOCKING], e.g. 1:enableAuxOutput:2,3:HARD (INDEX 1 = first x,y)."""
    fields = text.split(':')
    if len(fields) not in (3, 4):
        sys.exit(f"'{text}' is not INDEX:TYPE:OUTPUTS[:BLOCKING]")
    try:
        index = int(fields[0])
        outputs = [int(v) for v in fields[2].split(',')]
    except ValueError:
        sys.exit(f"'{text}': INDEX and OUTPUTS must be integers")
    blocking = fields[3].upper() if len(fields) == 4 else 'NONE'
    return index, action(fields[1], blocking, outputs_parameter(outputs))


def node(node_id: str, sequence_id: int, x: float, y: float, theta: float, map_id: str) -> dict:
    return {
        'nodeId': node_id,
        'sequenceId': sequence_id,
        'released': True,
        'nodePosition': {'x': x, 'y': y, 'theta': theta, 'mapId': map_id},
        'actions': [],
    }


def edge(sequence_id: int, start: dict, end: dict) -> dict:
    return {
        'edgeId': f'{start["nodeId"]}-{end["nodeId"]}',
        'sequenceId': sequence_id,
        'released': True,
        'startNodeId': start['nodeId'],
        'endNodeId': end['nodeId'],
        'actions': [],
    }


def parse_point(text: str):
    try:
        x, y = (float(v) for v in text.split(','))
    except ValueError:
        sys.exit(f"'{text}' is not x,y")
    return x, y


def send_order(master: Master, args):
    state = master.state()
    position = state.get('agvPosition') or {}
    map_id = position.get('mapId', '')
    points = [parse_point(p) for p in args.points]

    if args.from_node:
        # Stitch: the update starts at the last base node of the running order (VDA 5050 6.6.2).
        nodes = [n for n in state.get('nodeStates', []) if n['nodeId'] == args.from_node]
        last_sequence = max([n['sequenceId'] for n in nodes], default=state['lastNodeSequenceId'])
        start = node(args.from_node, last_sequence, 0.0, 0.0, 0.0, map_id)
        if nodes and nodes[0].get('nodePosition'):
            start['nodePosition'] = nodes[0]['nodePosition']
        first_sequence = last_sequence
    else:
        if not position.get('positionInitialized'):
            sys.exit('The rover reports no position (positionInitialized false).')
        start = node('start', 0, position['x'], position['y'], position['theta'], map_id)
        first_sequence = 0

    nodes, edges = [start], []
    for i, (x, y) in enumerate(points, start=1):
        sequence = first_sequence + 2 * i
        nodes.append(node(f'n{sequence}', sequence, x, y, 0.0, map_id))
        edges.append(edge(sequence - 1, nodes[-2], nodes[-1]))

    for index, node_action in (parse_node_action(a) for a in args.node_action):
        if not 0 <= index < len(nodes):
            sys.exit(f'--node-action index {index} is not a node of this order (0..{len(nodes) - 1})')
        nodes[index]['actions'].append(node_action)

    master.publish('order', {
        'orderId': args.order_id or f'order-{uuid.uuid4().hex[:8]}',
        'orderUpdateId': args.update,
        'nodes': nodes,
        'edges': edges,
    })


def summarize(topic: str, payload: dict) -> str:
    if topic == 'state':
        return (f"state #{payload['headerId']}: order={payload.get('orderId')!r} "
                f"last={payload.get('lastNodeId')!r} "
                f"nodes_left={[n['nodeId'] for n in payload.get('nodeStates', [])]} "
                f"driving={payload.get('driving')} paused={payload.get('paused')} "
                f"mode={payload.get('operatingMode')} "
                f"actions={[(a.get('actionType'), a['actionStatus'], a.get('resultDescription', '')) for a in payload.get('actionStates', [])]} "
                f"errors={[(e['errorType'], e.get('errorDescription')) for e in payload.get('errors', [])]}")
    if topic == 'connection':
        return f"connection: {payload.get('connectionState')}"
    return f'{topic}: {json.dumps(payload)}'


def watch(master: Master, seconds: float):
    deadline = None if seconds <= 0 else time.monotonic() + seconds
    for _ in itertools.count():
        for topic, payload in master.take_messages():
            print(summarize(topic, payload), flush=True)
        if deadline is not None and time.monotonic() > deadline:
            return
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--interface', default='uagv')
    parser.add_argument('--manufacturer', default='MechatronicsAcademy')
    parser.add_argument('--serial', default='rover_a1')
    parser.add_argument('--watch', type=float, default=5.0,
                        help='Seconds to print traffic after the command (0: until Ctrl-C).')
    commands = parser.add_subparsers(dest='command', required=True)

    commands.add_parser('watch')
    order = commands.add_parser('order')
    order.add_argument('points', nargs='+', help='x,y of each node after the start node')
    order.add_argument('--order-id', default='')
    order.add_argument('--update', type=int, default=0, help='orderUpdateId')
    order.add_argument('--from-node', default='',
                       help='Stitch: nodeId of the last base node of the running order')
    order.add_argument('--node-action', action='append', default=[],
                       help='INDEX:TYPE:OUTPUTS[:BLOCKING] (default NONE); INDEX 0 is the start '
                            'node, 1 the first x,y. Repeatable.')
    aux = commands.add_parser('aux')
    aux.add_argument('state', choices=['on', 'off'])
    aux.add_argument('outputs', nargs='+', type=int, help='Aux outputs 1..6')
    commands.add_parser('pause')
    commands.add_parser('resume')
    commands.add_parser('cancel')
    commands.add_parser('factsheet')
    drive_mode = commands.add_parser('drive-mode')
    drive_mode.add_argument('mode', choices=['MANUAL', 'AUTOMATIC'])

    args = parser.parse_args()
    master = Master(args)

    try:
        if args.command == 'order':
            send_order(master, args)
        elif args.command == 'aux':
            master.instant_action(
                'enableAuxOutput' if args.state == 'on' else 'disableAuxOutput',
                outputs_parameter(args.outputs))
        elif args.command == 'pause':
            master.instant_action('startPause')
        elif args.command == 'resume':
            master.instant_action('stopPause')
        elif args.command == 'cancel':
            master.instant_action('cancelOrder')
        elif args.command == 'factsheet':
            master.instant_action('factsheetRequest')
        elif args.command == 'drive-mode':
            master.instant_action('setDriveMode', [{'key': 'mode', 'value': args.mode}])

        watch(master, 0.0 if args.command == 'watch' else args.watch)
    except KeyboardInterrupt:
        pass
    finally:
        master.close()


if __name__ == '__main__':
    main()
