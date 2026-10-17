(() => {
  'use strict';
  const configured = window.OPENASC_CONFIG?.repository;
  let repository = null;
  if (configured && /^https:\/\/github\.com\/[\w.-]+\/[\w.-]+\/?$/.test(configured)) {
    repository = configured.replace(/\/$/, '');
  } else if (location.hostname.endsWith('.github.io')) {
    const owner = location.hostname.slice(0, -10);
    const name = location.pathname.split('/').filter(Boolean)[0] || `${owner}.github.io`;
    repository = `https://github.com/${owner}/${encodeURIComponent(name)}`;
  }
  if (repository) {
    document.querySelectorAll('[data-repo], [data-repo-path]').forEach(link => {
      link.href = repository + (link.dataset.repoPath || '');
    });
  }
  const button = document.querySelector('.copy');
  if (navigator.clipboard && window.isSecureContext) {
    button.hidden = false;
    button.addEventListener('click', async () => {
      const status = document.querySelector('#copy-status');
      try {
        await navigator.clipboard.writeText(document.querySelector('#scan-command').textContent);
        status.textContent = 'Команда скопирована.';
      } catch {
        status.textContent = 'Выделите команду и скопируйте её вручную.';
      }
    });
  }
})();
