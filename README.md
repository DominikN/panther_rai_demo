# panther_rai_demo

## How `src/panther_whoami` was created?

https://github.com/RobotecAI/rai/blob/development/docs/create_robots_whoami.md

## Quick Start

### update submodules

```bash
git submodule update --init --recursive
```

### `OPENAI_API_KEY` secret key setup

Create `.env` file in the root directory of this repo with the following content:

```bash
OPENAI_API_KEY=sk-xxxxxxxxx-yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
```

And paste here your own OpenAI API key.

### `LANGFUSE_*_KEY` secrets key setup

In the same `.env` file add your Public and Secret Keys for your project at https://cloud.langfuse.com/

```bash
LANGFUSE_PUBLIC_KEY=pk-lf-xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
LANGFUSE_SECRET_KEY=sk-lf-xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
```

### Building

```bash
docker compose build
```

### Launching

```bash
xhost +local:docker
docker compose up
```

### Accesing the Web HMI

Open http://localhost:8501 in a web browser.