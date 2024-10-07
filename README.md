# panther_rai_demo

## How `src/panther_whoami` was created?

https://github.com/RobotecAI/rai/blob/development/docs/create_robots_whoami.md

## Quick Start

### `OPENAI_API_KEY` secret key setup

Create `.env` file in the root directory of this repo with the following content:

```bash
OPENAI_API_KEY=sk-xxxxxxxxx-yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
```

And paste here your own OpenAI API key.

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