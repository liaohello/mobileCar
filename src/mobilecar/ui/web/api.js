const JSON_HEADERS = { "Content-Type": "application/json" };

export class ApiError extends Error {
  constructor(message, status = 0, details = null) {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.details = details;
  }
}

export class MobileCarApi {
  constructor({ baseUrl = "", fetchImpl = window.fetch.bind(window) } = {}) {
    this.baseUrl = baseUrl.replace(/\/$/, "");
    this.fetchImpl = fetchImpl;
  }

  url(path) {
    return `${this.baseUrl}${path}`;
  }

  async request(path, options = {}) {
    const response = await this.fetchImpl(this.url(path), {
      ...options,
      headers: { ...(options.body ? JSON_HEADERS : {}), ...(options.headers || {}) },
      cache: "no-store",
    });
    const contentType = response.headers.get("content-type") || "";
    const body = contentType.includes("application/json")
      ? await response.json()
      : await response.text();
    if (!response.ok) {
      const detail = typeof body === "object" && body ? body.detail || body.reason : body;
      throw new ApiError(
        typeof detail === "string" ? detail : `HTTP ${response.status}`,
        response.status,
        body,
      );
    }
    return body;
  }

  health() {
    return this.request("/api/v1/health");
  }

  capabilities() {
    return this.request("/api/v1/capabilities");
  }

  scene() {
    return this.request("/api/v1/scene");
  }

  snapshot() {
    return this.request("/api/v1/snapshot");
  }

  submitCommand(command) {
    return this.request("/api/v1/commands", {
      method: "POST",
      body: JSON.stringify(command),
    });
  }

  renewLease(sessionId) {
    return this.request("/api/v1/lease/renew", {
      method: "POST",
      body: JSON.stringify({ session_id: sessionId }),
    });
  }

  startEpisode(episode) {
    return this.request("/api/v1/episodes/start", {
      method: "POST",
      body: JSON.stringify(episode),
    });
  }

  stopEpisode(reason) {
    return this.request("/api/v1/episodes/stop", {
      method: "POST",
      body: JSON.stringify({ reason }),
    });
  }

  listEpisodes() {
    return this.request("/api/v1/episodes");
  }

  readEpisode(episodeId) {
    return this.request(`/api/v1/episodes/${encodeURIComponent(episodeId)}`);
  }

  validateEpisode(episodeId) {
    return this.request(`/api/v1/episodes/${encodeURIComponent(episodeId)}/validate`, {
      method: "POST",
    });
  }

  assessment(episodeId) {
    return this.request(`/api/v1/episodes/${encodeURIComponent(episodeId)}/assessment`);
  }

  eligibility(episodeId) {
    return this.request(`/api/v1/episodes/${encodeURIComponent(episodeId)}/eligibility`);
  }

  g4a1Status() {
    return this.request("/api/v1/g4a1/status");
  }

  g4a1Observation() {
    return this.request("/api/v1/g4a1/observation");
  }

  g4a1Permission(payload) {
    return this.request("/api/v1/g4a1/permission", {
      method: "POST",
      body: JSON.stringify(payload),
    });
  }

  g4a1Action(payload) {
    return this.request("/api/v1/g4a1/action", {
      method: "POST",
      body: JSON.stringify(payload),
    });
  }

  g4a1Revoke(reason) {
    return this.request("/api/v1/g4a1/revoke", {
      method: "POST",
      body: JSON.stringify({ reason, source: "test_injection" }),
    });
  }
}

export function randomId(prefix = "id") {
  if (globalThis.crypto?.randomUUID) {
    return `${prefix}_${globalThis.crypto.randomUUID()}`;
  }
  const random = Math.random().toString(36).slice(2);
  return `${prefix}_${Date.now().toString(36)}_${random}`;
}

export function episodeAssetUrl(episodeId, assetPath) {
  const encodedId = encodeURIComponent(episodeId);
  const normalized = String(assetPath)
    .split("/")
    .filter(Boolean)
    .map(encodeURIComponent)
    .join("/");
  return `/api/v1/episodes/${encodedId}/assets/${normalized}`;
}
