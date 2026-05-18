// Browser file store for .pp files using IndexedDB.
// The exported API stays promise-based so callers do not care about the backend.

export interface FileInfo {
  name: string;
  path: string;
  size: number;
  modified: number;
}

type StoredFile = {
  path: string;
  content: string;
  modified: number;
};

const DB_NAME = "pedro_visualizer_files";
const DB_VERSION = 1;
const STORE_NAME = "files";
const LEGACY_STORAGE_KEY = "pp_files";
const LEGACY_MIGRATION_KEY = "pp_files_indexeddb_migrated";

let dbPromise: Promise<IDBDatabase> | null = null;

function requestToPromise<T>(request: IDBRequest<T>): Promise<T> {
  return new Promise((resolve, reject) => {
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error || new Error("IndexedDB request failed."));
  });
}

function waitForTransaction(transaction: IDBTransaction): Promise<void> {
  return new Promise((resolve, reject) => {
    transaction.oncomplete = () => resolve();
    transaction.onerror = () =>
      reject(transaction.error || new Error("IndexedDB transaction failed."));
    transaction.onabort = () =>
      reject(transaction.error || new Error("IndexedDB transaction aborted."));
  });
}

function openDatabase(): Promise<IDBDatabase> {
  if (typeof indexedDB === "undefined") {
    return Promise.reject(new Error("IndexedDB is not available in this browser."));
  }

  return new Promise((resolve, reject) => {
    const request = indexedDB.open(DB_NAME, DB_VERSION);

    request.onupgradeneeded = () => {
      const db = request.result;
      if (!db.objectStoreNames.contains(STORE_NAME)) {
        db.createObjectStore(STORE_NAME, { keyPath: "path" });
      }
    };

    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error || new Error("Unable to open IndexedDB."));
  });
}

async function migrateLegacyLocalStorage(db: IDBDatabase) {
  if (typeof localStorage === "undefined") return;
  if (localStorage.getItem(LEGACY_MIGRATION_KEY) === "1") return;

  const raw = localStorage.getItem(LEGACY_STORAGE_KEY);
  if (!raw) {
    localStorage.setItem(LEGACY_MIGRATION_KEY, "1");
    return;
  }

  const files = JSON.parse(raw) as Record<string, { content: string; modified: number }>;
  const entries = Object.entries(files);
  if (entries.length === 0) {
    localStorage.setItem(LEGACY_MIGRATION_KEY, "1");
    return;
  }

  const transaction = db.transaction(STORE_NAME, "readwrite");
  const store = transaction.objectStore(STORE_NAME);
  entries.forEach(([path, file]) => {
    store.put({
      path,
      content: file.content,
      modified: Number(file.modified) || Date.now(),
    } satisfies StoredFile);
  });
  await waitForTransaction(transaction);
  localStorage.setItem(LEGACY_MIGRATION_KEY, "1");
}

async function getDb(): Promise<IDBDatabase> {
  if (!dbPromise) {
    dbPromise = openDatabase().then(async (db) => {
      await migrateLegacyLocalStorage(db);
      return db;
    });
  }
  return dbPromise;
}

function getLegacyFiles(): Record<string, { content: string; modified: number }> {
  if (typeof localStorage === "undefined") return {};
  const raw = localStorage.getItem(LEGACY_STORAGE_KEY);
  return raw ? JSON.parse(raw) : {};
}

function saveLegacyFiles(files: Record<string, { content: string; modified: number }>) {
  if (typeof localStorage === "undefined") {
    throw new Error("No browser storage backend is available.");
  }

  try {
    localStorage.setItem(LEGACY_STORAGE_KEY, JSON.stringify(files));
  } catch (error) {
    throw new Error(
      "Local browser storage is full. Save or export your route before making more changes.",
    );
  }
}

async function withLegacyFallback<T>(operation: () => Promise<T>, fallback: () => T): Promise<T> {
  try {
    return await operation();
  } catch (error) {
    if (typeof indexedDB !== "undefined") {
      throw error;
    }
    return fallback();
  }
}

export async function listFiles(): Promise<FileInfo[]> {
  return withLegacyFallback(
    async () => {
      const db = await getDb();
      const files = await requestToPromise<StoredFile[]>(
        db.transaction(STORE_NAME, "readonly").objectStore(STORE_NAME).getAll(),
      );
      return files.map((file) => ({
        name: file.path,
        path: file.path,
        size: file.content.length,
        modified: file.modified,
      }));
    },
    () => {
      const files = getLegacyFiles();
      return Object.entries(files).map(([name, { content, modified }]) => ({
        name,
        path: name,
        size: content.length,
        modified,
      }));
    },
  );
}

export async function readFile(path: string): Promise<string> {
  return withLegacyFallback(
    async () => {
      const db = await getDb();
      const file = await requestToPromise<StoredFile | undefined>(
        db.transaction(STORE_NAME, "readonly").objectStore(STORE_NAME).get(path),
      );
      if (!file) throw new Error("File not found");
      return file.content;
    },
    () => {
      const files = getLegacyFiles();
      if (!(path in files)) throw new Error("File not found");
      return files[path].content;
    },
  );
}

export async function writeFile(path: string, content: string): Promise<boolean> {
  return withLegacyFallback(
    async () => {
      const db = await getDb();
      const transaction = db.transaction(STORE_NAME, "readwrite");
      transaction.objectStore(STORE_NAME).put({
        path,
        content,
        modified: Date.now(),
      } satisfies StoredFile);
      await waitForTransaction(transaction);
      return true;
    },
    () => {
      const files = getLegacyFiles();
      files[path] = { content, modified: Date.now() };
      saveLegacyFiles(files);
      return true;
    },
  );
}

export async function deleteFile(path: string): Promise<boolean> {
  return withLegacyFallback(
    async () => {
      const exists = await fileExists(path);
      if (!exists) return false;
      const db = await getDb();
      const transaction = db.transaction(STORE_NAME, "readwrite");
      transaction.objectStore(STORE_NAME).delete(path);
      await waitForTransaction(transaction);
      return true;
    },
    () => {
      const files = getLegacyFiles();
      if (!(path in files)) return false;
      delete files[path];
      saveLegacyFiles(files);
      return true;
    },
  );
}

export async function fileExists(path: string): Promise<boolean> {
  return withLegacyFallback(
    async () => {
      const db = await getDb();
      const key = await requestToPromise<IDBValidKey | undefined>(
        db.transaction(STORE_NAME, "readonly").objectStore(STORE_NAME).getKey(path),
      );
      return key !== undefined;
    },
    () => path in getLegacyFiles(),
  );
}

export async function renameFile(
  oldPath: string,
  newPath: string,
): Promise<{ success: boolean; newPath: string }> {
  return withLegacyFallback(
    async () => {
      const db = await getDb();
      const existingNewPath = await requestToPromise<IDBValidKey | undefined>(
        db.transaction(STORE_NAME, "readonly").objectStore(STORE_NAME).getKey(newPath),
      );
      if (existingNewPath !== undefined) return { success: false, newPath };

      const existing = await requestToPromise<StoredFile | undefined>(
        db.transaction(STORE_NAME, "readonly").objectStore(STORE_NAME).get(oldPath),
      );
      if (!existing) return { success: false, newPath: oldPath };

      const transaction = db.transaction(STORE_NAME, "readwrite");
      const store = transaction.objectStore(STORE_NAME);
      store.put({ ...existing, path: newPath, modified: Date.now() });
      store.delete(oldPath);
      await waitForTransaction(transaction);
      return { success: true, newPath };
    },
    () => {
      const files = getLegacyFiles();
      if (!(oldPath in files)) return { success: false, newPath: oldPath };
      if (newPath in files) return { success: false, newPath };
      files[newPath] = { ...files[oldPath], modified: Date.now() };
      delete files[oldPath];
      saveLegacyFiles(files);
      return { success: true, newPath };
    },
  );
}

export async function getDirectoryStats(): Promise<{
  totalFiles: number;
  totalSize: number;
  lastModified: number;
}> {
  return withLegacyFallback(
    async () => {
      const files = await listFiles();
      return {
        totalFiles: files.length,
        totalSize: files.reduce((sum, file) => sum + file.size, 0),
        lastModified: files.reduce((max, file) => Math.max(max, file.modified), 0),
      };
    },
    () => {
      const files = Object.values(getLegacyFiles());
      return {
        totalFiles: files.length,
        totalSize: files.reduce((sum, file) => sum + file.content.length, 0),
        lastModified: files.reduce((max, file) => Math.max(max, file.modified), 0),
      };
    },
  );
}
