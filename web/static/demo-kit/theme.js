// Demo kit v1 (contract: /demo-kit/v1/): sets a demo's color scheme. Load it
// as a classic script in <head>, before tokens.css, so the scheme is set before
// first paint. The first match wins:
// 1. the embedding portfolio page's <html data-theme>, which it follows live;
// 2. a #light or #dark URL fragment;
// 3. the scheme the visitor chose with a demo's toggle or the portfolio's
//    switcher, both saved as localStorage "theme" on this same origin;
// 4. dark, the portfolio's default.
// It sets <html data-theme> and, inside a frame, <html data-embedded>, exposes
// window.demoTheme() ("light" or "dark") and window.demoSetTheme(scheme), and
// fires a "demo-theme" event on window when the scheme changes, for canvas apps
// that draw their own colors. A click on any [data-demo-theme-toggle] element
// switches the scheme, as the portfolio's switcher does.
(function () {
    "use strict";
    var root = document.documentElement;
    var embedded = window.parent !== window;

    function scheme(value) {
        return value === "light" || value === "dark" ? value : null;
    }
    function hostTheme() {
        try {
            return embedded ? scheme(window.parent.document.documentElement.getAttribute("data-theme")) : null;
        } catch (error) {
            return null; // A cross-origin host cannot be read.
        }
    }
    function saved() {
        try {
            return scheme(window.localStorage.getItem("theme"));
        } catch (error) {
            return null; // Storage may be blocked.
        }
    }
    function current() {
        return scheme(root.getAttribute("data-theme")) || "dark";
    }

    var chosen = null; // Kept here too, in case storage is blocked.
    var last = null;
    function apply() {
        root.setAttribute("data-theme", hostTheme() || scheme(window.location.hash.slice(1)) || chosen || saved() || "dark");
        var toggles = document.querySelectorAll("[data-demo-theme-toggle]");
        for (var i = 0; i < toggles.length; i++) toggles[i].setAttribute("aria-pressed", String(current() === "dark"));
        if (current() !== last) {
            last = current();
            window.dispatchEvent(new CustomEvent("demo-theme", { detail: last }));
        }
    }
    function set(value) {
        if (!scheme(value)) return;
        chosen = value;
        try {
            window.localStorage.setItem("theme", value);
        } catch (error) {
            // Storage may be blocked; the choice then lasts for this page.
        }
        // A fragment passed by a portfolio link would otherwise outrank it.
        if (scheme(window.location.hash.slice(1))) {
            window.history.replaceState(window.history.state, "", window.location.pathname + window.location.search);
        }
        apply();
    }

    if (embedded) root.setAttribute("data-embedded", "");
    window.demoTheme = current;
    window.demoSetTheme = set;
    apply();
    document.addEventListener("DOMContentLoaded", apply);
    document.addEventListener("click", function (event) {
        if (event.target.closest && event.target.closest("[data-demo-theme-toggle]")) {
            set(current() === "dark" ? "light" : "dark");
        }
    });
    window.addEventListener("hashchange", apply);
    // A choice saved in another tab, of the portfolio or a demo.
    window.addEventListener("storage", function (event) {
        if (event.key === "theme" || event.key === null) {
            chosen = null;
            apply();
        }
    });
    try {
        if (embedded) {
            new MutationObserver(apply).observe(window.parent.document.documentElement, {
                attributes: true,
                attributeFilter: ["data-theme"],
            });
        }
    } catch (error) {
        // A cross-origin host keeps the fragment, saved choice or default.
    }
})();
