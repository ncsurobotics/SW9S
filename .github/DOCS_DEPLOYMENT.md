# Documentation deployment

The `Deploy documentation` workflow builds Rust documentation on pull requests
and publishes it from `main` to:

https://docs.aquapackrobotics.org/software/sw9s/

It can also be run manually on `main`. Other branches build an artifact without
deploying. Private items are included because SW9S is a binary crate.

## One-time setup

1. In **ncsurobotics/SW9S → Settings → Pages**, select **GitHub Actions** as
   the build and deployment source.
2. Set the custom domain to **docs.aquapackrobotics.org** in those same settings.
3. In Cloudflare's DNS records for `aquapackrobotics.org`, add:

   | Type | Name | Target | Proxy status |
   | --- | --- | --- | --- |
   | CNAME | `docs` | `ncsurobotics.github.io` | DNS only |

   The target is a hostname, without a repository name or URL path. Keep the
   existing apex and `www` records unchanged.
4. Once GitHub's DNS check and certificate provisioning finish, enable
   **Enforce HTTPS** in SW9S's Pages settings.
5. Ensure the `github-pages` environment permits deployment from `main`, then
   merge the workflow into `main` or run it manually from that branch.

The workflow uses GitHub's built-in token and OIDC permissions; no Cloudflare
token or cross-repository credentials are required. With Actions-based Pages,
the custom domain is configured in repository settings, not a `CNAME` file.

## Site ownership and layout

SW9S owns the Pages site for the **entire docs subdomain**. The public website
continues to deploy separately from `ncsurobotics/ncsurobotics.github.io`.
Each docs deployment replaces the docs site, so additional projects must be
assembled into the same deployment or moved to a shared documentation repository.
Do not publish other projects independently to this same hostname.

The complete `target/doc` tree is copied under `/software/`, placing the crate
index at `/software/sw9s/` while retaining the relative paths to Rustdoc's CSS,
JavaScript, search index, and source pages. The site root links to the SW9S docs.

References: [GitHub Pages workflows](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages),
[custom domains](https://docs.github.com/en/pages/configuring-a-custom-domain-for-your-github-pages-site/managing-a-custom-domain-for-your-github-pages-site),
and [Cloudflare subdomain records](https://developers.cloudflare.com/dns/manage-dns-records/how-to/create-subdomain/).
