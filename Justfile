module := "stdb-physics"

server:
    spacetime publish -p spacetime_physics {{module}} -y -c

delete:
    spacetime delete {{module}} -y

client:
    cargo run --bin client

bindings:
    spacetime generate --lang rust --out-dir ./client/src/module_bindings --project-path spacetime_physics

logs:
    spacetime logs {{module}}

messages:
    spacetime logs {{module}} --format json | jq -r '.message'

sc: server client

sm sleep='1':
    just server 
    sleep {{sleep}}
    just messages

all: server bindings client
