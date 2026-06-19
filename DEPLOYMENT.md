# Deploying OrbitForge to Cloudflare Pages

The CI workflow (`.github/workflows/deploy.yml`) already builds the WASM
binary and the web bundle and deploys them on every push to `main`. It is
complete and requires no code changes. What's left is account-side setup
on Cloudflare and GitHub.

## Steps

1. **Create the Cloudflare Pages project**
   Cloudflare dashboard → Workers & Pages → Create → Pages. Name the
   project `orbitforge`. A direct-upload project is fine — the GitHub
   Actions workflow pushes the build itself, so connecting Git to
   Cloudflare is not required.

2. **Generate a Cloudflare API token**
   Cloudflare dashboard → My Profile → API Tokens → Create Token.
   Use the "Edit Cloudflare Workers" template, or a custom token with
   **Account → Cloudflare Pages → Edit** permission.

3. **Find your Cloudflare Account ID**
   Dashboard → any domain, or the Workers & Pages overview page → Account
   ID is shown in the right sidebar.

4. **Add both as GitHub repo secrets**
   Repo → Settings → Secrets and variables → Actions → New repository
   secret:
   - `CLOUDFLARE_API_TOKEN`
   - `CLOUDFLARE_ACCOUNT_ID`

5. **Push to `main`**
   This triggers `deploy.yml`, which builds the WASM binary via
   `scripts/build_wasm.sh`, builds the web bundle via `npm run build`, and
   deploys both to the `orbitforge` Pages project.

## Caveat for the first run

The Emscripten build (`scripts/build_wasm.sh`) has never been run end to
end — there's no local Emscripten toolchain in this dev environment, so
it's only ever been validated by reading the script, not by executing it.
The first CI run on `deploy.yml` is the first real test of that build
step. Watch the Actions log on that push in case the Emscripten flags
need adjustment.
