# FRC 4734 Driver Dashboard

Vue 3 driver dashboard that displays live robot data and lets the driver select an autonomous routine. It connects to the robot via a small Node.js bridge that talks to the roboRIO's NetworkTables (NT4) and relays data to the browser over WebSocket.

No changes to the Java robot code are required. The dashboard reads from the same SmartDashboard and Shuffleboard/Driver keys that the existing [DriverDashboard](src/main/java/frc/robot/dashboard/DriverDashboard.java) writes to, and writes the auto selection to the same SendableChooser the robot uses.

## Prerequisites

- Node.js 18+ (LTS recommended)
- Robot (or simulation) publishing to NetworkTables on the same network

## Quick start

1. **Install dependencies** (frontend and bridge):

   ```bash
   cd driver-dashboard
   npm install
   cd server
   npm install
   cd ..
   ```

2. **Start the bridge** (connects to the robot's NetworkTables and serves WebSocket updates):

   ```bash
   npm run server
   ```

   Leave this running. It listens on port **5811** by default.

3. **Start the Vue app** (development):

   ```bash
   npm run dev
   ```

   Open http://localhost:5174 (or the port Vite prints). The dashboard will connect to the bridge at `ws://localhost:5811`.

4. **Production**: Build and run the bridge (it can serve the built Vue app if you add static serving, or run them separately):

   ```bash
   npm run build
   npm run server
   ```

   Then open the built files (e.g. with a static file server or by configuring the bridge to serve the `dist` folder).

## Configuration

The bridge reads these environment variables:

| Variable | Default | Description |
|----------|---------|--------------|
| `TEAM_NUMBER` | `4734` | FRC team number; used to resolve the roboRIO address (e.g. `10.47.34.2` or `roborio-4734-frc.local`). |
| `ROBOT_IP` | (none) | If set, connect to this IP instead of using team number. Example: `10.47.34.2`. |
| `NT_PORT` | `5810` | NetworkTables 4 server port on the robot. |
| `DASHBOARD_WS_PORT` | `5811` | Port the bridge's WebSocket server listens on. |

Examples:

- Windows (PowerShell): `$env:TEAM_NUMBER="4734"; node server/index.mjs`
- Linux/macOS: `TEAM_NUMBER=4734 node server/index.mjs`
- Use a specific IP: `ROBOT_IP=10.47.34.2 node server/index.mjs`

## What the dashboard shows

- **Connection**: Green = dashboard and robot connected; yellow = dashboard connected, waiting for robot; red = dashboard disconnected.
- **Match timer**: From the Driver tab (robot-formatted string with phase).
- **Game pieces, Shooter, Vision, Alliance, Current action**: From Shuffleboard Driver tab.
- **Autonomous**: Dropdown of options from the robot's SendableChooser; selection is written back to NetworkTables so the robot runs the chosen auto.
- **Robot pose / Vision pose**: From SmartDashboard (fused pose and vision-estimated pose).

## SendableChooser / auto selector

The bridge subscribes to:

- `SmartDashboard/Auto Mode (manager)/options` (string array)
- `SmartDashboard/Auto Mode (manager)/selected` (string)

When the driver selects an auto in the Vue app, the bridge publishes the chosen string to the `selected` topic. If the auto selector does not update the robot, the NT4 topic names may differ (e.g. different path or `.controllable`). Use AdvantageScope or another NT client to inspect the exact topic names and update the bridge if needed.

## Running with Shuffleboard

You can run this dashboard and Shuffleboard at the same time. Both are clients of NetworkTables; the robot remains the single source of truth.

## Scripts

| Script | Description |
|--------|-------------|
| `npm run dev` | Start Vite dev server (frontend only; start the bridge separately). |
| `npm run build` | Build the Vue app for production. |
| `npm run server` | Start the Node bridge (WebSocket + NT client). |
| `npm run start` | Build the Vue app, then start the bridge (bridge does not serve static files; use a separate server or add static middleware if needed). |
