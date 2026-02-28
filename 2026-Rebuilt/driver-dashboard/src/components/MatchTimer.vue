<script setup lang="ts">
defineProps<{
  matchTimer: string
}>()
</script>

<template>
  <div class="match-timer">
    <span class="value">{{ matchTimer }}</span>
  </div>
</template>

<style scoped>
.match-timer {
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 1.5rem 2rem;
  background: var(--panel-bg, #1a1a2e);
  border-radius: 12px;
  border: 2px solid var(--border, #334155);
}

.value {
  font-family: 'JetBrains Mono', monospace;
  font-size: 3rem;
  font-weight: 700;
  color: var(--text, #f1f5f9);
  letter-spacing: 0.05em;
}
</style>
