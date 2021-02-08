## Dependências
A integração da UnBall com o FIRASim depende da biblioteca compartilhada `vss.so` disponível para ser compilada no repositório `FIRA_Client`.

- Clone o repositório `FIRA_Client`, na branch `master`.
- Compile a biblioteca compartilhada seguindo as instruções do repositório.
- Copie este arquivo para a pasta `lib` no repositório `Integration`
- Vá para a pasta `src/client/protobuf` e execute o `protobuf.sh`
- Volte para a raíz do projeto e execute `pip3 install -r requirements.txt`

## Execução
Para executar o sistema rode `python3 src/main.py [cor do time]`
