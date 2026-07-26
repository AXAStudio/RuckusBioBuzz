/**
 * Drag-to-scrub for numeric fields.
 *
 * Typing a coordinate is fine for a first guess and bad for the tenth: nudging
 * a waypoint an inch at a time means select-all, retype, tab, look at the field,
 * repeat. Dragging sideways moves it directly, and the field redraws as it goes,
 * so a position can be dialled in by eye.
 *
 * The drag lives on the field's **label** — the `X:`, `Y:`, `Start:` next to the
 * box — not on the box itself. Putting it on the input meant every click into a
 * field to type had to be told apart from the start of a drag, and a stray few
 * pixels while clicking would nudge the value. The label has no other job, so
 * there is nothing to disambiguate: labels drag, boxes type.
 *
 * Only literals scrub. A field showing an expression is owned by a variable, and
 * writing a number over it would silently break the link — scrub the variable
 * itself in the Variables tab and everything reading it follows.
 */

/** Pixels of travel before a press counts as a drag rather than a click. */
export const SCRUB_THRESHOLD_PX = 3;

/** How much one pixel is worth, before modifiers. */
export const DEFAULT_SCRUB_STEP = 0.05;

export interface ScrubModifiers {
  /** Shift: ten times as fast, for crossing the field. */
  coarse?: boolean;
  /** Alt: a tenth as fast, for the last hundredth of an inch. */
  fine?: boolean;
}

/** The step one pixel of drag is worth under the held modifiers. */
export function scrubStepFor(step: number, modifiers: ScrubModifiers = {}): number {
  const base = Number(step) > 0 ? Number(step) : DEFAULT_SCRUB_STEP;
  if (modifiers.coarse) return base * 10;
  if (modifiers.fine) return base / 10;
  return base;
}

/**
 * The value after dragging `deltaPixels` from where the press started.
 *
 * Always computed from the value at press time rather than accumulated, so a
 * drag out and back lands exactly where it started instead of drifting by the
 * rounding of every frame in between.
 */
export function scrubValue(
  startValue: number,
  deltaPixels: number,
  step: number,
  modifiers: ScrubModifiers = {},
): number {
  const perPixel = scrubStepFor(step, modifiers);
  const raw = startValue + deltaPixels * perPixel;

  // Land on multiples of the step in use, so the number stays readable and a
  // slow drag does not produce 4.300000000000001.
  const snapped = Math.round(raw / perPixel) * perPixel;
  return roundToStep(snapped, perPixel);
}

/** Trims float noise to the precision the step implies. */
export function roundToStep(value: number, step: number): number {
  const decimals = decimalsFor(step);
  const factor = Math.pow(10, decimals);
  return Math.round(value * factor) / factor;
}

/** How many decimals a step size needs to be written exactly. */
export function decimalsFor(step: number): number {
  if (!(step > 0)) return 2;
  // 0.05 needs two, 0.5 needs one, 5 needs none. Cap it so a step that came out
  // of a division cannot ask for twelve.
  const decimals = Math.ceil(-Math.log10(step));
  return Math.max(0, Math.min(6, decimals));
}

/**
 * Whether a field's text is a plain number rather than an expression.
 *
 * An empty field counts, and scrubs from zero: a coordinate that has not been
 * typed yet is still a coordinate.
 */
export function isScrubbableLiteral(text: string | number | null | undefined): boolean {
  const value = String(text ?? "").trim();
  if (!value) return true;
  return /^-?\d*\.?\d*$/.test(value) && value !== "-" && value !== ".";
}

/** Formats a scrubbed value for writing back into a text field. */
export function formatScrubbed(value: number, step: number): string {
  const decimals = decimalsFor(step);
  // Keep it short: 12.5 rather than 12.50, but 12 rather than 12.00.
  return String(Number(value.toFixed(decimals)));
}

/* -------------------------------------------------------------------------
 * The action
 * ---------------------------------------------------------------------- */

export interface ScrubOptions {
  /** What the field currently shows. Read at press time, not during the drag. */
  value: string | number | null | undefined;
  /** What one pixel is worth. Inches by default; headings want more. */
  step?: number;
  disabled?: boolean;
  /** Called on every frame of the drag, so the field follows live. */
  onInput: (next: string) => void;
  /** Called once when the drag ends, so it lands as one undo entry. */
  onCommit?: (next: string) => void;
}

const SCRUB_HINT = "Drag sideways to adjust. Shift for coarse, Alt for fine.";

/**
 * Makes a label drag its field's number.
 *
 * Attach to the `X:` / `Start:` / `Deg:` label sitting beside an input, passing
 * the same value and callbacks the input itself gets.
 */
export function scrubbable(node: HTMLElement, options: ScrubOptions) {
  let current = options;
  let armed = false;
  let dragging = false;
  let startX = 0;
  let startValue = 0;
  const originalTitle = node.getAttribute("title");

  const enabled = () => !current.disabled && isScrubbableLiteral(current.value);

  function paint() {
    const on = enabled();
    node.style.cursor = on ? "ew-resize" : "";
    // Without this a drag selects the label's own text on the way past.
    node.style.userSelect = on ? "none" : "";
    node.style.touchAction = on ? "none" : "";

    if (on) {
      node.setAttribute("title", originalTitle ? `${originalTitle} ${SCRUB_HINT}` : SCRUB_HINT);
    } else if (originalTitle) {
      node.setAttribute("title", originalTitle);
    } else {
      node.removeAttribute("title");
    }
  }

  function stepWith(event: PointerEvent): number {
    return scrubStepFor(current.step ?? DEFAULT_SCRUB_STEP, {
      coarse: event.shiftKey,
      fine: event.altKey,
    });
  }

  function onPointerDown(event: PointerEvent) {
    if (!enabled() || event.button !== 0) return;
    armed = true;
    dragging = false;
    startX = event.clientX;
    startValue = Number(current.value) || 0;
    // Claim the pointer up front: the label is small and the drag will leave it
    // almost immediately.
    node.setPointerCapture(event.pointerId);
    event.preventDefault();
  }

  function onPointerMove(event: PointerEvent) {
    if (!armed) return;

    const delta = event.clientX - startX;
    if (!dragging) {
      if (Math.abs(delta) < SCRUB_THRESHOLD_PX) return;
      dragging = true;
      node.classList.add("ring-1", "ring-sky-400", "rounded");
    }

    event.preventDefault();

    const step = current.step ?? DEFAULT_SCRUB_STEP;
    const next = scrubValue(startValue, delta, step, {
      coarse: event.shiftKey,
      fine: event.altKey,
    });
    const text = formatScrubbed(next, stepWith(event));
    if (text === String(current.value)) return;

    current.onInput(text);
  }

  function onPointerUp(event: PointerEvent) {
    if (!armed) return;
    armed = false;

    if (node.hasPointerCapture(event.pointerId)) {
      node.releasePointerCapture(event.pointerId);
    }
    if (!dragging) return;

    dragging = false;
    node.classList.remove("ring-1", "ring-sky-400", "rounded");
    current.onCommit?.(String(current.value ?? ""));
  }

  node.addEventListener("pointerdown", onPointerDown);
  node.addEventListener("pointermove", onPointerMove);
  node.addEventListener("pointerup", onPointerUp);
  node.addEventListener("pointercancel", onPointerUp);
  paint();

  return {
    update(next: ScrubOptions) {
      // The value changes on every frame of a drag; the press-time value is held
      // separately so that does not disturb it.
      current = next;
      if (!dragging) paint();
    },
    destroy() {
      node.removeEventListener("pointerdown", onPointerDown);
      node.removeEventListener("pointermove", onPointerMove);
      node.removeEventListener("pointerup", onPointerUp);
      node.removeEventListener("pointercancel", onPointerUp);
    },
  };
}
