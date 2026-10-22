// The Lite UI's link to the backend (/api/*): the session token, JSON calls and refusals.
// A refusal comes back as {error, hint, code}; call() throws it as an Error with those fields.
// A job's view ({id, state, error, hint, …}) is a result even when the job failed: it is returned,
// so whoever watches the job reads its state and says why it stopped.

let token = null;
let onLoginLost = null;

export const session = {
  get token() { return token; },
  set(t) { token = t || null; },
  onLost(fn) { onLoginLost = fn; },
};

export async function call(method, path, body, { raw = false, keepalive = false, as = token } = {}) {
  const headers = { Accept: raw ? '*/*' : 'application/json' };
  if (as) headers['X-Lite-Token'] = as;                    // `as`: another session's token (the one a Lock ends)
  let payload;
  if (body instanceof Blob || body instanceof ArrayBuffer) {
    headers['Content-Type'] = 'application/octet-stream';
    payload = body;
  } else if (body !== undefined) {
    headers['Content-Type'] = 'application/json';
    payload = JSON.stringify(body);
  }
  let r;
  try {
    r = await fetch(path, { method, headers, body: payload, keepalive: keepalive && (!payload || payload.length < 60000) });
  } catch (_) {
    throw Object.assign(new Error('Lite cannot reach its server.'), { hint: 'Is ALIX Studio Lite still running?', code: 'offline' });
  }
  const type = r.headers.get('Content-Type') || '';
  // a file's bytes (the server marks every file it sends: an S2 sequence is a JSON file, which was parsed here and
  // saved as an empty file, the lab 22 Oct); a refusal is JSON without the mark
  if (raw && (r.headers.get('X-Lite-Raw') || !type.startsWith('application/json'))) return r.blob();
  const d = await r.json().catch(() => ({ error: 'The server sent an answer Lite cannot read.' }));
  if (d && d.error && !(d.id && d.state)) {
    if (d.code === 'login' && onLoginLost) onLoginLost();
    throw Object.assign(new Error(d.error), { hint: d.hint, code: d.code });
  }
  return d;
}

export const get = (path, opts) => call('GET', path, undefined, opts);
export const post = (path, body = {}, opts) => call('POST', path, body, opts);
export const put = (path, body, opts) => call('PUT', path, body, opts);
export const del = (path, opts) => call('DELETE', path, undefined, opts);

// A refusal as one line for the page's toast: what happened, then what to do.
export function say(err) {
  return err.hint ? `${err.message} ${err.hint}` : err.message;
}
