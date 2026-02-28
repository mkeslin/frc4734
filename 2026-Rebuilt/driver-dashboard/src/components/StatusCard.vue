<script setup lang="ts">
defineProps<{
  title: string
  value: string
}>()
</script>

<template>
  <div class="status-card">
    <div class="title">{{ title }}</div>
    <div class="value">{{ value }}</div>
  </div>
</template>

<style scoped>
.status-card {
  padding: 0.75rem 1rem;
  background: var(--panel-bg, #1a1a2e);
  border-radius: 8px;
  border: 1px solid var(--border, #334155);
}

.title {
  font-family: 'Outfit', sans-serif;
  font-size: 0.75rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--text-muted, #94a3b8);
  margin-bottom: 0.25rem;
}

.value {
  font-family: 'JetBrains Mono', monospace;
  font-size: 1rem;
  font-weight: 600;
  color: var(--text, #f1f5f9);
}
</style>
