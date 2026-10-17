# Публикация сайта openASC

Сайт находится в `site/`. Workflow загружает **только этот каталог**, а не локальную установку игры.

1. Создайте GitHub-репозиторий для проекта, например `openASC`.
2. Загрузите новые исходные файлы и документацию. Каталог `assettocorsa/` должен остаться локальным: он уже указан в `.gitignore`. Перед первым коммитом проверьте `git status --short`.
3. Откройте **Settings → Pages → Build and deployment → Source → GitHub Actions**.
4. Запушьте файлы в `main` или `master`, либо вручную запустите **Deploy GitHub Pages** во вкладке Actions на основной ветке репозитория.
5. После успешного задания `deploy` откройте URL в его environment `github-pages`.

Для репозитория `OWNER/openASC` обычный адрес будет `https://OWNER.github.io/openASC/`. Для репозитория `OWNER/OWNER.github.io` — `https://OWNER.github.io/`.

Workflow использует `configure-pages@v5`, `upload-pages-artifact@v4` и `deploy-pages@v4`, согласно [документации GitHub](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages).

## Ссылки на исходники

На github.io сайт автоматически выводит адрес репозитория из имени хоста и первого сегмента пути. Для собственного домена или нестандартного размещения укажите полный HTTPS-адрес репозитория в `site/config.js`. Локально кнопка исходников ведёт к инструкции публикации на странице; вымышленные адреса не используются.

## Локальная проверка

```powershell
python -m http.server 4173 --directory site --bind 127.0.0.1
```

Откройте `http://127.0.0.1:4173`. Статус сайта описывает только реализованное; обновляйте его по мере появления проверенных возможностей.
