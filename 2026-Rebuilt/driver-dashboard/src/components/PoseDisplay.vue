<script setup lang="ts">
defineProps<{
  robotPoseX: number
  robotPoseY: number
  robotPoseRotationDeg: number
  visionPoseX: number
  visionPoseY: number
  visionPoseRotationDeg: number
}>()

function fmt(n: number): string {
  if (n !== n) return '—'
  return n.toFixed(2)
}
</script>

<template>
  <div class="pose-display">
    <div class="section">
      <div class="title">Robot pose</div>
      <div class="row">
        <span>X {{ fmt(robotPoseX) }}</span>
        <span>Y {{ fmt(robotPoseY) }}</span>
        <span>° {{ fmt(robotPoseRotationDeg) }}</span>
      </div>
    </div>
    <div class="section">
      <div class="title">Vision pose</div>
      <div class="row">
        <span>X {{ fmt(visionPoseX) }}</span>
        <span>Y {{ fmt(visionPoseY) }}</span>
        <span>° {{ fmt(visionPoseRotationDeg) }}</span>
      </div>
    </div>
  </div>
</template>

<style scoped>
.pose-display {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  padding: 0.75rem 1rem;
  background: var(--panel-bg, #1a1a2e);
  border-radius: 8px;
  border: 1px solid var(--border, #334155);
}

.section .title {
  font-family: 'Outfit', sans-serif;
  font-size: 0.7rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--text-muted, #94a3b8);
  margin-bottom: 0.25rem;
}

.row {
  display: flex;
  gap: 1rem;
  font-family: 'JetBrains Mono', monospace;
  font-size: 0.9rem;
  color: var(--text, #f1f5f9);
}
</style>
