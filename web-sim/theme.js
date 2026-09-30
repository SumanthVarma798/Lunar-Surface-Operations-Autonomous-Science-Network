/* Apply the saved theme before first paint. Without a saved choice the
   prefers-color-scheme media query in tokens.css decides (dark is the default). */
(function () {
  try {
    var saved = localStorage.getItem("lsoas:theme");
    if (saved === "light" || saved === "dark") {
      document.documentElement.setAttribute("data-theme", saved);
    }
  } catch (_err) {
    // Storage can be unavailable (private mode); fall back to the system theme.
  }
})();
