/**
 * Composable for WebSocket connection to the dashboard bridge.
 * Holds reactive state for all NT values and connection status.
 */

import { ref, reactive, onMounted, onUnmounted } from 'vue'

const WS_URL =
  typeof window !== 'undefined'
    ? (window.location.protocol === 'https:' ? 'wss:' : 'ws:') +
      '//' +
      window.location.hostname +
      ':5811'
    : ''

export interface DashboardState {
  wsConnected: boolean
  robotConnected: boolean
  matchTimer: string
  gamePieces: string
  shooterStatus: string
  visionStatus: string
  currentAction: string
  alliance: string
  robotPoseX: number
  robotPoseY: number
  robotPoseRotationDeg: number
  visionPoseX: number
  visionPoseY: number
  visionPoseRotationDeg: number
  autoOptions: string[]
  autoSelected: string
}

const defaultState: DashboardState = {
  wsConnected: false,
  robotConnected: false,
  matchTimer: '--:--',
  gamePieces: '-',
  shooterStatus: '-',
  visionStatus: '-',
  currentAction: '-',
  alliance: '-',
  robotPoseX: 0,
  robotPoseY: 0,
  robotPoseRotationDeg: 0,
  visionPoseX: 0,
  visionPoseY: 0,
  visionPoseRotationDeg: 0,
  autoOptions: [],
  autoSelected: '',
}

export function useDashboardSocket() {
  const state = reactive<DashboardState>({ ...defaultState })
  const ws = ref<WebSocket | null>(null)
  const reconnectTimer = ref<ReturnType<typeof setTimeout> | null>(null)

  function connect() {
    if (ws.value?.readyState === WebSocket.OPEN) return
    const url = WS_URL
    if (!url) return
    try {
      const socket = new WebSocket(url)
      socket.onopen = () => {
        state.wsConnected = true
      }
      socket.onclose = () => {
        state.wsConnected = false
        ws.value = null
        reconnectTimer.value = setTimeout(connect, 2000)
      }
      socket.onerror = () => {
        socket.close()
      }
      socket.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data as string)
          if (msg.key && Object.prototype.hasOwnProperty.call(state, msg.key)) {
            ;(state as Record<string, unknown>)[msg.key] = msg.value
          }
        } catch (_) {
          /* ignore */
        }
      }
      ws.value = socket
    } catch (_) {
      reconnectTimer.value = setTimeout(connect, 2000)
    }
  }

  function disconnect() {
    if (reconnectTimer.value) {
      clearTimeout(reconnectTimer.value)
      reconnectTimer.value = null
    }
    if (ws.value) {
      ws.value.close()
      ws.value = null
    }
    state.wsConnected = false
  }

  function setAutoSelected(value: string) {
    if (ws.value?.readyState === WebSocket.OPEN) {
      ws.value.send(JSON.stringify({ setAutoSelected: value }))
    }
    state.autoSelected = value
  }

  onMounted(() => {
    connect()
  })

  onUnmounted(() => {
    disconnect()
  })

  return { state, connect, disconnect, setAutoSelected }
}
