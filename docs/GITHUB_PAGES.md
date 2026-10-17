# GitHub Pages

The live site is [openasc-project.github.io/openASC](https://openasc-project.github.io/openASC/). The source repository is [openASC-project/openASC](https://github.com/openASC-project/openASC).

## Deployment

The `Deploy GitHub Pages` workflow uploads **only `site/`**, never the local installation or reports. It runs on pushes affecting site files or the deployment workflow on `main` or `master`, and only deploys from the repository's default branch. A manual run is also available in GitHub Actions.

1. Edit documentation in `docs/` or overview content in `web/overview.html`.
2. Run `python tools/build_site.py` to regenerate HTML.
3. Run `python tools/build_site.py --check` and the checks in the development guide.
4. Commit the source and generated pages, then push to the default branch.
5. Check the deployment result and its `github-pages` environment URL in Actions.

## Configure a fork

1. Open Settings → Pages → Build and deployment.
2. Set Source to GitHub Actions.
3. Update repository links in `web/layout.html`, `web/overview.html`, `tools/build_site.py`, and documentation for your fork.
4. Regenerate the site and push to the default branch.

All page and asset links are relative, so the site works beneath a repository path. A repository called `OWNER/openASC` normally publishes at `https://OWNER.github.io/openASC/`.

The workflow uses `configure-pages@v5`, `upload-pages-artifact@v4`, and `deploy-pages@v4`, following the [GitHub documentation](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages).

## Preview locally

```sh
python -m http.server 4173 --directory site --bind 127.0.0.1
```

Open [localhost](http://127.0.0.1:4173/). Review the overview and documentation at desktop and phone widths before publishing. Site claims must match the currently implemented and tested features.
