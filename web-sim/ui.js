/* ═══════════════════════════════════════════════════════
   LSOAS - UI helpers
   Theme toggle, segmented controls, dialog focus trap, range fills,
   task-family and state icon maps. No simulation logic lives here.
   ═══════════════════════════════════════════════════════ */

(function () {
  "use strict";

  const $ = (sel, root = document) => root.querySelector(sel);
  const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

  const reducedMotionQuery = window.matchMedia(
    "(prefers-reduced-motion: reduce)",
  );
  const reducedMotion = () => reducedMotionQuery.matches;

  /* One icon per task family, used in the task list, rover cards, command log,
     legend and task-type picker so the mapping is learnable. */
  const TASK_FAMILIES = {
    movement: { icon: "ph-path", label: "Movement" },
    science: { icon: "ph-flask", label: "Science" },
    digging: { icon: "ph-shovel", label: "Digging" },
    pushing: { icon: "ph-bulldozer", label: "Pushing" },
    photo: { icon: "ph-camera", label: "Imaging" },
    "sample-handling": { icon: "ph-test-tube", label: "Sample handling" },
  };

  /* Weight follows meaning: duotone at rest, fill when something is wrong or
     needs attention. The spinner stays regular so it reads as an arc. */
  const ROVER_STATES = {
    IDLE: { cls: "ph-duotone ph-pause-circle", label: "Idle" },
    EXECUTING: { cls: "ph ph-circle-notch", label: "Running" },
    SAFE_MODE: { cls: "ph-fill ph-shield-warning", label: "Safe mode" },
    ERROR: { cls: "ph-fill ph-warning-octagon", label: "Fault" },
    UNKNOWN: { cls: "ph ph-question", label: "Unknown" },
  };

  function taskFamily(type) {
    return (
      TASK_FAMILIES[String(type || "").toLowerCase()] || TASK_FAMILIES.movement
    );
  }

  function roverState(state) {
    return ROVER_STATES[String(state || "UNKNOWN").toUpperCase()] || ROVER_STATES.UNKNOWN;
  }

  const EASE = "cubic-bezier(0.16, 1, 0.3, 1)";

  /* Icon swap feedback: a short scale and fade so a state change reads as a morph. */
  function pop(el) {
    if (!el || !el.animate || reducedMotion()) return;
    el.animate(
      [
        { transform: "scale(0.55)", opacity: 0.2 },
        { transform: "scale(1)", opacity: 1 },
      ],
      { duration: 380, easing: EASE },
    );
  }

  /* Fault attention: three pulses, then stop. Never an infinite loop. */
  function pulse(el) {
    if (!el || !el.animate || reducedMotion()) return;
    el.animate(
      [
        { transform: "scale(1)" },
        { transform: "scale(1.35)" },
        { transform: "scale(1)" },
      ],
      { duration: 700, iterations: 3, easing: EASE },
    );
  }

  /* Run a DOM update inside a View Transition when the browser supports it and
     motion is allowed; otherwise just run it. */
  function transition(update, { name } = {}) {
    if (
      typeof document.startViewTransition !== "function" ||
      reducedMotion() ||
      document.hidden
    ) {
      update();
      return Promise.resolve();
    }
    if (name) document.documentElement.dataset.vt = name;
    const vt = document.startViewTransition(update);
    const done = () => {
      if (name) delete document.documentElement.dataset.vt;
    };
    vt.finished.then(done, done);
    return vt.finished.catch(() => {});
  }

  /* ─── Segmented controls ─── */
  function segSet(seg, index) {
    if (!seg) return;
    const buttons = $$(".seg-btn", seg);
    const i = Math.max(0, Math.min(index, buttons.length - 1));
    seg.style.setProperty("--seg-i", String(i));
    buttons.forEach((btn, n) => {
      const on = n === i;
      btn.classList.toggle("active", on);
      btn.setAttribute("aria-pressed", on ? "true" : "false");
    });
  }

  function initSegmented(seg, onChange) {
    if (!seg) return;
    const buttons = $$(".seg-btn", seg);
    buttons.forEach((btn, index) => {
      btn.addEventListener("click", () => {
        segSet(seg, index);
        if (onChange) onChange(btn, index);
      });
      btn.addEventListener("keydown", (event) => {
        if (event.key !== "ArrowRight" && event.key !== "ArrowLeft") return;
        event.preventDefault();
        const step = event.key === "ArrowRight" ? 1 : -1;
        const next = (index + step + buttons.length) % buttons.length;
        buttons[next].focus();
        buttons[next].click();
      });
    });
  }

  /* ─── Theme ─── */
  function effectiveTheme() {
    const explicit = document.documentElement.getAttribute("data-theme");
    if (explicit === "light" || explicit === "dark") return explicit;
    return window.matchMedia("(prefers-color-scheme: light)").matches
      ? "light"
      : "dark";
  }

  function paintThemeButton(btn) {
    const isLight = effectiveTheme() === "light";
    const icon = $("i", btn);
    if (icon) icon.className = isLight ? "ph ph-moon" : "ph ph-sun";
    const label = isLight ? "Switch to dark theme" : "Switch to light theme";
    btn.setAttribute("aria-label", label);
    btn.setAttribute("title", label);
    document.dispatchEvent(
      new CustomEvent("lsoas:theme", { detail: { theme: effectiveTheme() } }),
    );
  }

  function initTheme() {
    const btn = $("#btn-theme");
    if (!btn) return;
    paintThemeButton(btn);
    btn.addEventListener("click", () => {
      const next = effectiveTheme() === "light" ? "dark" : "light";
      transition(() => {
        document.documentElement.setAttribute("data-theme", next);
        paintThemeButton(btn);
      }, { name: "theme" });
      try {
        localStorage.setItem("lsoas:theme", next);
      } catch (_err) {
        // Preference just will not persist.
      }
    });
    window
      .matchMedia("(prefers-color-scheme: light)")
      .addEventListener("change", () => paintThemeButton(btn));
  }

  /* ─── Dialog: focus trap, inert background, Escape ─── */
  const dialogState = { el: null, opener: null, onClose: null, onKey: null };
  const INERT_ROOTS = ["#top-bar", ".view-nav", "#app"];
  const FOCUSABLE =
    'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

  function openDialog(el, { onClose, initialFocus } = {}) {
    if (!el) return;
    dialogState.el = el;
    dialogState.opener = document.activeElement;
    dialogState.onClose = onClose || null;
    INERT_ROOTS.forEach((sel) => {
      const node = $(sel);
      if (node) node.inert = true;
    });
    dialogState.onKey = (event) => {
      if (event.key === "Escape") {
        event.preventDefault();
        if (dialogState.onClose) dialogState.onClose();
        return;
      }
      if (event.key !== "Tab") return;
      const items = $$(FOCUSABLE, el).filter(
        (n) => n.offsetParent !== null || n === document.activeElement,
      );
      if (items.length === 0) return;
      const first = items[0];
      const last = items[items.length - 1];
      if (event.shiftKey && document.activeElement === first) {
        event.preventDefault();
        last.focus();
      } else if (!event.shiftKey && document.activeElement === last) {
        event.preventDefault();
        first.focus();
      } else if (!el.contains(document.activeElement)) {
        event.preventDefault();
        first.focus();
      }
    };
    document.addEventListener("keydown", dialogState.onKey, true);
    requestAnimationFrame(() => {
      const target = initialFocus || $(FOCUSABLE, el);
      if (target) target.focus({ preventScroll: true });
    });
  }

  function closeDialog() {
    if (!dialogState.el) return;
    document.removeEventListener("keydown", dialogState.onKey, true);
    INERT_ROOTS.forEach((sel) => {
      const node = $(sel);
      if (node) node.inert = false;
    });
    const opener = dialogState.opener;
    dialogState.el = null;
    dialogState.onClose = null;
    dialogState.onKey = null;
    dialogState.opener = null;
    if (opener && typeof opener.focus === "function" && document.contains(opener)) {
      opener.focus({ preventScroll: true });
    }
  }

  /* ─── Range fill (paints the filled part of a slider track) ─── */
  function initRanges() {
    $$("input[type='range']").forEach((input) => {
      const paint = () => {
        const min = Number(input.min || 0);
        const max = Number(input.max || 100);
        const pct = ((Number(input.value) - min) / (max - min)) * 100;
        input.style.setProperty("--pct", `${pct}%`);
      };
      input.addEventListener("input", paint);
      paint();
    });
  }

  /* ─── Dashboard view switcher (narrow screens) ─── */
  function initViewSwitch() {
    const seg = $("#view-switch");
    const app = $("#app");
    if (!seg || !app) return;
    initSegmented(seg, (btn) => {
      const view = btn.dataset.viewTarget;
      transition(() => {
        app.dataset.view = view;
        window.scrollTo({ top: 0 });
      }, { name: "view" });
    });
  }

  /* The header wraps to two rows on narrow screens. Keep --header-h equal to its
     real height so sticky elements below it line up. */
  function initHeaderHeight() {
    const header = $("#top-bar");
    if (!header || typeof ResizeObserver !== "function") return;
    const sync = () => {
      document.documentElement.style.setProperty(
        "--header-h",
        `${Math.round(header.getBoundingClientRect().height)}px`,
      );
    };
    new ResizeObserver(sync).observe(header);
    sync();
  }

  window.LSOASUI = {
    reducedMotion,
    taskFamily,
    roverState,
    taskFamilies: TASK_FAMILIES,
    pop,
    pulse,
    transition,
    segSet,
    initSegmented,
    openDialog,
    closeDialog,
  };

  initHeaderHeight();
  initTheme();
  initRanges();
  initViewSwitch();
  initSegmented($("#time-controls"));
})();
