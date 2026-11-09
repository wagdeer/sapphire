const header = document.querySelector("[data-header]");
const nav = document.querySelector("[data-nav]");
const navToggle = document.querySelector("[data-nav-toggle]");

const updateHeader = () => {
  header?.classList.toggle("is-scrolled", window.scrollY > 16);
};

updateHeader();
window.addEventListener("scroll", updateHeader, { passive: true });

navToggle?.addEventListener("click", () => {
  const open = nav?.classList.toggle("is-open") ?? false;
  navToggle.setAttribute("aria-expanded", String(open));
});

nav?.querySelectorAll("a").forEach((link) => {
  link.addEventListener("click", () => {
    nav.classList.remove("is-open");
    navToggle?.setAttribute("aria-expanded", "false");
  });
});

const revealItems = document.querySelectorAll(".reveal");
if ("IntersectionObserver" in window) {
  const revealObserver = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (!entry.isIntersecting) return;
      entry.target.classList.add("is-visible");
      revealObserver.unobserve(entry.target);
    });
  }, { threshold: 0.12, rootMargin: "0px 0px -40px" });

  revealItems.forEach((item) => revealObserver.observe(item));
} else {
  revealItems.forEach((item) => item.classList.add("is-visible"));
}

const demoVideo = document.querySelector("[data-demo-video]");
const demoTabs = document.querySelectorAll(".demo-tab");
const demoTitle = document.querySelector("[data-demo-title]");
const demoMeta = document.querySelector("[data-demo-meta]");

const loadDemo = (tab) => {
  if (!demoVideo || !tab) return;

  demoTabs.forEach((item) => {
    const active = item === tab;
    item.classList.toggle("is-active", active);
    item.setAttribute("aria-selected", String(active));
  });

  demoVideo.pause();
  demoVideo.poster = tab.dataset.poster;
  const source = demoVideo.querySelector("source");
  if (source) source.src = tab.dataset.video;
  demoVideo.load();
  demoVideo.play().catch(() => {});

  if (demoTitle) demoTitle.textContent = tab.dataset.title;
  if (demoMeta) demoMeta.textContent = tab.dataset.meta;
};

demoTabs.forEach((tab) => {
  tab.addEventListener("click", () => loadDemo(tab));
});

const heroVideo = document.querySelector("[data-hero-video]");
if (heroVideo && "IntersectionObserver" in window) {
  const videoObserver = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        heroVideo.play().catch(() => {});
      } else {
        heroVideo.pause();
      }
    });
  }, { threshold: 0.1 });

  videoObserver.observe(heroVideo);
}

document.querySelectorAll("[data-copy-target]").forEach((button) => {
  button.addEventListener("click", async () => {
    const target = document.getElementById(button.dataset.copyTarget);
    if (!target) return;

    const text = target.innerText;
    try {
      await navigator.clipboard.writeText(text);
      const oldLabel = button.textContent;
      button.textContent = "Copied";
      button.classList.add("is-copied");
      window.setTimeout(() => {
        button.textContent = oldLabel;
        button.classList.remove("is-copied");
      }, 1600);
    } catch {
      button.textContent = "Select code";
    }
  });
});

const year = document.querySelector("[data-year]");
if (year) year.textContent = String(new Date().getFullYear());
