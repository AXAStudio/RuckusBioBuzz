import type {
  Point,
  Line,
  Shape,
  SequenceItem,
  Settings,
  PathChain,
  PoseVariable,
  PathVariable,
  NumberVariable,
} from "../types";
import { writable } from "svelte/store";

export type AppState = {
  startPoint: Point;
  lines: Line[];
  shapes: Shape[];
  sequence: SequenceItem[];
  settings: Settings;
  pathChains: PathChain[];
  poseVariables: PoseVariable[];
  pathVariables: PathVariable[];
  numberVariables: NumberVariable[];
};

type PersistedHistory = {
  version: 1;
  undoStack: AppState[];
  redoStack: AppState[];
  lastHash: string;
  savedAt: number;
};

const DEFAULT_STORAGE_KEY = "pedro_visualizer_history_v1";
const DEFAULT_PERSISTED_SIZE = 50;

function deepClone<T>(obj: T): T {
  return JSON.parse(JSON.stringify(obj));
}

function canUseLocalStorage() {
  return typeof localStorage !== "undefined";
}

export function createHistory(
  maxSize = 200,
  storageKey = DEFAULT_STORAGE_KEY,
  persistedSize = DEFAULT_PERSISTED_SIZE,
) {
  let undoStack: AppState[] = [];
  let redoStack: AppState[] = [];
  let lastHash = "";

  // Create writable stores to trigger reactivity
  const canUndoStore = writable(false);
  const canRedoStore = writable(false);

  function updateStores() {
    canUndoStore.set(undoStack.length > 1);
    canRedoStore.set(redoStack.length > 0);
  }

  function hash(state: AppState): string {
    // Stable hash via JSON string; sufficient for change detection here
    return JSON.stringify(state);
  }

  function trimStacksForMemory() {
    if (undoStack.length > maxSize) {
      undoStack = undoStack.slice(-maxSize);
    }
    if (redoStack.length > maxSize) {
      redoStack = redoStack.slice(-maxSize);
    }
  }

  function persistedSnapshot(): PersistedHistory {
    const limit = Math.max(1, Math.min(maxSize, persistedSize));
    return {
      version: 1,
      undoStack: undoStack.slice(-limit),
      redoStack: redoStack.slice(-limit),
      lastHash,
      savedAt: Date.now(),
    };
  }

  function persist() {
    if (!canUseLocalStorage()) return;

    let snapshot = persistedSnapshot();
    while (snapshot.undoStack.length > 0) {
      try {
        localStorage.setItem(storageKey, JSON.stringify(snapshot));
        return;
      } catch (error) {
        // Keep the newest recovery points if localStorage is nearly full.
        snapshot = {
          ...snapshot,
          undoStack: snapshot.undoStack.slice(1),
          redoStack: snapshot.redoStack.slice(1),
        };
      }
    }
  }

  function restore() {
    if (!canUseLocalStorage()) return;

    try {
      const raw = localStorage.getItem(storageKey);
      if (!raw) return;
      const parsed = JSON.parse(raw) as PersistedHistory;
      if (parsed.version !== 1 || !Array.isArray(parsed.undoStack)) return;

      undoStack = parsed.undoStack.map(deepClone).slice(-maxSize);
      redoStack = Array.isArray(parsed.redoStack)
        ? parsed.redoStack.map(deepClone).slice(-maxSize)
        : [];
      lastHash =
        typeof parsed.lastHash === "string"
          ? parsed.lastHash
          : undoStack.length
            ? hash(undoStack[undoStack.length - 1])
            : "";
      updateStores();
    } catch (error) {
      // A corrupt recovery history should not prevent the visualizer from loading.
      undoStack = [];
      redoStack = [];
      lastHash = "";
    }
  }

  restore();

  function record(state: AppState) {
    const snapshot = deepClone(state);
    const currentHash = hash(snapshot);
    if (currentHash === lastHash) {
      // No meaningful change
      return;
    }
    undoStack.push(snapshot);
    lastHash = currentHash;
    // Cap stack size
    trimStacksForMemory();
    // Clear redo on new action
    redoStack = [];
    updateStores();
    persist();
  }

  function canUndo() {
    return undoStack.length > 1; // keep initial state; require at least one prior state
  }

  function canRedo() {
    return redoStack.length > 0;
  }

  function undo(): AppState | null {
    if (!canUndo()) return null;
    const current = undoStack.pop()!; // current state to redo
    const prev = undoStack[undoStack.length - 1];
    redoStack.push(current);
    lastHash = hash(prev);
    trimStacksForMemory();
    updateStores();
    persist();
    return deepClone(prev);
  }

  function redo(): AppState | null {
    if (!canRedo()) return null;
    const next = redoStack.pop()!;
    undoStack.push(next);
    lastHash = hash(next);
    trimStacksForMemory();
    updateStores();
    persist();
    return deepClone(next);
  }

  function peek(): AppState | null {
    if (undoStack.length === 0) return null;
    return deepClone(undoStack[undoStack.length - 1]);
  }

  return {
    record,
    undo,
    redo,
    canUndo,
    canRedo,
    peek,
    canUndoStore,
    canRedoStore,
  };
}
