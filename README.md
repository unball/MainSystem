## Dependências
A integração da UnBall com o FIRASim depende da biblioteca compartilhada `vss.so` disponível para ser compilada no repositório `FIRA_Client`.

- Clone o repositório `FIRA_Client`, vá a branch `feature_python_ctypes` e vá para a pasta `cpp-client`.
- Compile a biblioteca compartilhada seguindo as instruções do repositório.
- Copie este arquivo para a pasta `lib` no repositório `Integration`

## Execução
Para executar o sistema rode `python3 src/main.py`