(() => {
  'use strict';
  const root = document.documentElement;
  root.classList.add('js');
  const themeButton = document.querySelector('.theme-toggle');
  let storedTheme;
  try { storedTheme = localStorage.getItem('openasc-theme'); } catch { /* Storage may be unavailable. */ }
  function applyTheme(theme) {
    root.dataset.theme = theme;
    themeButton.setAttribute('aria-label', `Switch to ${theme === 'dark' ? 'light' : 'dark'} theme`);
  }
  applyTheme(['light', 'dark'].includes(storedTheme) ? storedTheme : 'dark');
  themeButton.hidden = false;
  themeButton.addEventListener('click', () => {
    const theme = root.dataset.theme === 'dark' ? 'light' : 'dark';
    applyTheme(theme);
    storedTheme = theme;
    try { localStorage.setItem('openasc-theme', theme); } catch { /* Theme still works for this page. */ }
  });

  const sidebar = document.querySelector('.sidebar');
  const menu = document.querySelector('.menu-button');
  const close = document.querySelector('.drawer-close');
  const backdrop = document.querySelector('.drawer-backdrop');
  const narrow = matchMedia('(max-width: 760px)');
  function setOpen(open, restoreFocus = true) {
    sidebar.classList.toggle('is-open', open);
    menu.setAttribute('aria-expanded', String(open));
    backdrop.hidden = !open;
    sidebar.inert = narrow.matches && !open;
    document.body.style.overflow = open ? 'hidden' : '';
    if (open) close.focus();
    else if (restoreFocus && narrow.matches) menu.focus();
  }
  setOpen(false, false);
  menu.addEventListener('click', () => setOpen(menu.getAttribute('aria-expanded') !== 'true'));
  close.addEventListener('click', () => setOpen(false));
  backdrop.addEventListener('click', () => setOpen(false));
  narrow.addEventListener('change', () => setOpen(false, false));
  sidebar.addEventListener('keydown', event => {
    if (!narrow.matches || menu.getAttribute('aria-expanded') !== 'true') return;
    if (event.key === 'Escape') {
      event.preventDefault();
      setOpen(false);
    } else if (event.key === 'Tab') {
      const elements = [...sidebar.querySelectorAll('a, button')];
      const first = elements[0], last = elements.at(-1);
      if (event.shiftKey && document.activeElement === first) {
        event.preventDefault(); last.focus();
      } else if (!event.shiftKey && document.activeElement === last) {
        event.preventDefault(); first.focus();
      }
    }
  });
})();
