<script setup lang="ts">
defineProps<{
  wsConnected: boolean
  robotConnected: boolean
}>()
</script>

<template>
  <div class="connection-bar">
    <span
      class="dot"
      :class="{
        green: wsConnected && robotConnected,
        yellow: wsConnected && !robotConnected,
        red: !wsConnected,
      }"
    />
    <span class="label">
      {{ !wsConnected ? 'Dashboard disconnected' : robotConnected ? 'Robot connected' : 'Waiting for robot' }}
    </span>
  </div>
</template>

<style scoped>
.connection-bar {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem 1rem;
  font-family: 'JetBrains Mono', monospace;
  font-size: 0.875rem;
  background: var(--panel-bg, #1a1a2e);
  border-radius: 6px;
}

.dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  flex-shrink: 0;
}

.dot.green {
  background: #22c55e;
  box-shadow: 0 0 8px #22c55e;
}

.dot.yellow {
  background: #eab308;
  box-shadow: 0 0 8px #eab308;
}

.dot.red {
  background: #ef4444;
  box-shadow: 0 0 8px #ef4444;
}

.label {
  color: var(--text-muted, #94a3b8);
}
</style>
