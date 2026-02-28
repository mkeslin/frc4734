<script setup lang="ts">
import { computed } from 'vue'

const props = defineProps<{
  options: string[]
  selected: string
}>()

const emit = defineEmits<{
  (e: 'update:selected', value: string): void
}>()

const selectedModel = computed({
  get: () => props.selected,
  set: (v: string) => emit('update:selected', v),
})

const displayOptions = computed(() => {
  const o = props.options
  if (!o?.length) return [{ label: 'No autos', value: '' }]
  return o.map((name) => ({ label: name, value: name }))
})
</script>

<template>
  <div class="auto-selector">
    <label class="label">Autonomous</label>
    <select v-model="selectedModel" class="select">
      <option v-for="opt in displayOptions" :key="opt.value" :value="opt.value">
        {{ opt.label }}
      </option>
    </select>
  </div>
</template>

<style scoped>
.auto-selector {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.label {
  font-family: 'Outfit', sans-serif;
  font-size: 0.75rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--text-muted, #94a3b8);
}

.select {
  font-family: 'JetBrains Mono', monospace;
  font-size: 1rem;
  padding: 0.75rem 1rem;
  background: var(--panel-bg, #1a1a2e);
  border: 2px solid var(--border, #334155);
  border-radius: 8px;
  color: var(--text, #f1f5f9);
  cursor: pointer;
}

.select:focus {
  outline: none;
  border-color: var(--accent, #3b82f6);
}
</style>
