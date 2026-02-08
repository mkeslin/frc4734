/**
 * NetworkTables bridge: connects to the roboRIO's NT4 server, subscribes to
 * SmartDashboard and Shuffleboard/Driver keys, and relays updates to Vue
 * clients over WebSocket. Also writes auto selector choice back to NT.
 */

import { WebSocketServer } from 'ws';
import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';

const WS_PORT = parseInt(process.env.DASHBOARD_WS_PORT || '5811', 10);
const TEAM_NUMBER = parseInt(process.env.TEAM_NUMBER || '4734', 10);
const ROBOT_IP = process.env.ROBOT_IP;
const NT_PORT = parseInt(process.env.NT_PORT || '5810', 10);

const nt =
  ROBOT_IP
    ? NetworkTables.getInstanceByURI(ROBOT_IP, NT_PORT)
    : NetworkTables.getInstanceByTeam(TEAM_NUMBER, NT_PORT);

const wss = new WebSocketServer({ port: WS_PORT });

function broadcast(msg) {
  const data = JSON.stringify(msg);
  for (const client of wss.clients) {
    if (client.readyState === 1) client.send(data);
  }
}

function ntKey(path) {
  return path.startsWith('/') ? path.slice(1) : path;
}

const topics = {
  matchTimer: nt.createTopic(ntKey('Shuffleboard/Driver/Match Timer'), NetworkTablesTypeInfos.kString, '--:--'),
  gamePieces: nt.createTopic(ntKey('Shuffleboard/Driver/Game Pieces'), NetworkTablesTypeInfos.kString, '-'),
  shooterStatus: nt.createTopic(ntKey('Shuffleboard/Driver/Shooter Status'), NetworkTablesTypeInfos.kString, '-'),
  visionStatus: nt.createTopic(ntKey('Shuffleboard/Driver/Vision Status'), NetworkTablesTypeInfos.kString, '-'),
  currentAction: nt.createTopic(ntKey('Shuffleboard/Driver/Current Action'), NetworkTablesTypeInfos.kString, '-'),
  alliance: nt.createTopic(ntKey('Shuffleboard/Driver/Alliance'), NetworkTablesTypeInfos.kString, '-'),
  robotPoseX: nt.createTopic(ntKey('SmartDashboard/Robot/PoseX'), NetworkTablesTypeInfos.kDouble, 0),
  robotPoseY: nt.createTopic(ntKey('SmartDashboard/Robot/PoseY'), NetworkTablesTypeInfos.kDouble, 0),
  robotPoseRotationDeg: nt.createTopic(ntKey('SmartDashboard/Robot/PoseRotationDeg'), NetworkTablesTypeInfos.kDouble, 0),
  visionPoseX: nt.createTopic(ntKey('SmartDashboard/Vision/EstimatedPoseX'), NetworkTablesTypeInfos.kDouble, 0),
  visionPoseY: nt.createTopic(ntKey('SmartDashboard/Vision/EstimatedPoseY'), NetworkTablesTypeInfos.kDouble, 0),
  visionPoseRotationDeg: nt.createTopic(ntKey('SmartDashboard/Vision/EstimatedPoseRotationDeg'), NetworkTablesTypeInfos.kDouble, 0),
};

for (const [name, topic] of Object.entries(topics)) {
  topic.subscribe((value) => {
    broadcast({ key: name, value });
  });
}

const chooserPrefix = ntKey('SmartDashboard/Auto Mode (manager)');
const chooserSelectedTopic = nt.createTopic(`${chooserPrefix}/selected`, NetworkTablesTypeInfos.kString, '');
const chooserOptionsTopic = nt.createTopic(`${chooserPrefix}/options`, NetworkTablesTypeInfos.kStringArray, []);

chooserSelectedTopic.subscribe((value) => broadcast({ key: 'autoSelected', value: value ?? '' }));
chooserOptionsTopic.subscribe((value) => broadcast({ key: 'autoOptions', value: value ?? [] }));

nt.addRobotConnectionListener((connected) => {
  broadcast({ key: 'robotConnected', value: connected });
}, true);

wss.on('connection', (ws) => {
  const snapshot = {
    matchTimer: topics.matchTimer.getValue(),
    gamePieces: topics.gamePieces.getValue(),
    shooterStatus: topics.shooterStatus.getValue(),
    visionStatus: topics.visionStatus.getValue(),
    currentAction: topics.currentAction.getValue(),
    alliance: topics.alliance.getValue(),
    robotPoseX: topics.robotPoseX.getValue(),
    robotPoseY: topics.robotPoseY.getValue(),
    robotPoseRotationDeg: topics.robotPoseRotationDeg.getValue(),
    visionPoseX: topics.visionPoseX.getValue(),
    visionPoseY: topics.visionPoseY.getValue(),
    visionPoseRotationDeg: topics.visionPoseRotationDeg.getValue(),
    autoSelected: chooserSelectedTopic.getValue(),
    autoOptions: chooserOptionsTopic.getValue(),
    robotConnected: nt.isRobotConnected(),
  };
  for (const [key, value] of Object.entries(snapshot)) {
    const v = value ?? (key === 'autoOptions' ? [] : key === 'robotConnected' ? false : '');
    ws.send(JSON.stringify({ key, value: v }));
  }

  ws.on('message', (raw) => {
    try {
      const msg = JSON.parse(raw.toString());
      if (msg.setAutoSelected != null) {
        chooserSelectedTopic.publish().then(() => {
          chooserSelectedTopic.setValue(String(msg.setAutoSelected));
        }).catch(() => {});
      }
    } catch (_) { /* ignore */ }
  });
});

console.log(`Dashboard bridge: WS on port ${WS_PORT}, NT team=${TEAM_NUMBER}${ROBOT_IP ? ` uri=${ROBOT_IP}` : ''} port=${NT_PORT}`);
