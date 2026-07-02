/** Tiny DOM helper: h('div.cls#id', {attrs}, ...children) */
export function h(spec, attrs = {}, ...children) {
  const [tag, ...rest] = spec.split(/(?=[.#])/);
  const el = document.createElement(tag || 'div');
  for (const part of rest) {
    if (part.startsWith('.')) el.classList.add(part.slice(1));
    if (part.startsWith('#')) el.id = part.slice(1);
  }
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'text') el.textContent = v;
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else el.setAttribute(k, v);
  }
  for (const c of children) {
    if (c == null) continue;
    el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return el;
}

/** Wire press-and-hold behaviour (mouse + touch) onto a button. */
export function hold(btn, onStart, onEnd) {
  const start = (e) => { e.preventDefault(); onStart(); };
  const end = () => onEnd();
  btn.addEventListener('pointerdown', start);
  btn.addEventListener('pointerup', end);
  btn.addEventListener('pointerleave', end);
  btn.addEventListener('pointercancel', end);
  btn.addEventListener('contextmenu', (e) => e.preventDefault());
  return btn;
}
