<script setup lang="ts">
import ConnectionBar from './components/ConnectionBar.vue'
import MatchTimer from './components/MatchTimer.vue'
import StatusCard from './components/StatusCard.vue'
import AutoSelector from './components/AutoSelector.vue'
import PoseDisplay from './components/PoseDisplay.vue'
import { useDashboardSocket } from './composables/useDashboardSocket'

const { state, setAutoSelected } = useDashboardSocket()
</script>

<template>
  <div class="app">
    <header class="header">
      <h1 class="title">FRC 4734 Driver</h1>
      <ConnectionBar :ws-connected="state.wsConnected" :robot-connected="state.robotConnected" />
    </header>

    <main class="main">
      <section class="timer-section">
        <MatchTimer :match-timer="state.matchTimer" />
      </section>

      <section class="status-grid">
        <StatusCard title="Game Pieces" :value="state.gamePieces" />
        <StatusCard title="Shooter" :value="state.shooterStatus" />
        <StatusCard title="Vision" :value="state.visionStatus" />
        <StatusCard title="Alliance" :value="state.alliance" />
        <StatusCard title="Current Action" :value="state.currentAction" class="span-2" />
      </section>

      <section class="auto-section">
        <AutoSelector
          :options="state.autoOptions"
          :selected="state.autoSelected"
          @update:selected="setAutoSelected"
        />
      </section>

      <section class="pose-section">
        <PoseDisplay
          :robot-pose-x="state.robotPoseX"
          :robot-pose-y="state.robotPoseY"
          :robot-pose-rotation-deg="state.robotPoseRotationDeg"
          :vision-pose-x="state.visionPoseX"
          :vision-pose-y="state.visionPoseY"
          :vision-pose-rotation-deg="state.visionPoseRotationDeg"
        />
      </section>
    </main>
  </div>
</template>

<style>
:root {
  --bg: #0f172a;
  --panel-bg: #1e293b;
  --border: #334155;
  --text: #f1f5f9;
  --text-muted: #94a3b8;
  --accent: #3b82f6;
}

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font-family: 'Outfit', sans-serif;
}

.app {
  min-height: 100vh;
  padding: 1rem;
  max-width: 1200px;
  margin: 0 auto;
}

.header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
  margin-bottom: 1.5rem;
  flex-wrap: wrap;
}

.title {
  font-size: 1.5rem;
  font-weight: 700;
  margin: 0;
}

.main {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.timer-section {
  width: 100%;
}

.status-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
}

.status-grid .span-2 {
  grid-column: span 2;
}

.auto-section {
  max-width: 400px;
}

.pose-section {
  max-width: 400px;
}
</style>
