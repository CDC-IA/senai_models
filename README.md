# Senai Models

Este repositório contém modelos de robôs em formato URDF e arquivos de configuração do `robot_state_publisher` juntamente com arquivos de lançamento (launch files) para ROS 2. Estes recursos são projetados para facilitar a simulação e manipulação de robôs em ambientes ROS 2.

## Estrutura do Repositório

A estrutura do repositório está organizada da seguinte forma:

```
senai_models/
├── models/
│   └── robots/
│       └── model/
│           ├── config/
│           ├── meshes/
│           └── urdf/
│               └── model.urdf
├── config/
│   └── robot_state_publisher.yaml
├── launch/
│   └── model_rsp.launch.py
├── worlds/
│   └── ??
├── README.md
├── CMakeLists.txt
└── package.xml
```

- **urdf/**: Contém os arquivos URDF (Unified Robot Description Format) que descrevem a estrutura dos robôs.
- **config/**: Contém arquivos de configuração para o `robot_state_publisher`.
- **launch/**: Contém os arquivos de lançamento para iniciar os nodos necessários.
- **package.xml**: Arquivo de metadados do pacote ROS 2.

## Pré-requisitos

Antes de utilizar este repositório, certifique-se de ter o ROS 2 instalado em seu sistema. Você pode seguir as instruções de instalação no site oficial do [ROS 2](https://docs.ros.org/en/foxy/Installation.html).

## Como Usar

1. **Clone o Repositório**

   Clone este repositório em seu espaço de trabalho ROS 2:

   ```bash
   git clone https://github.com/seu_usuario/senai_models.git
   cd senai_models
   ```

2. **Construir o Pacote**

   Certifique-se de estar no diretório raiz do seu espaço de trabalho ROS 2 e então construa o pacote:

   ```bash
   colcon build
   ```

3. **Fonte o Ambiente**

   Após a construção, fonte o ambiente para garantir que os novos pacotes estejam disponíveis:

   ```bash
   source install/setup.bash
   ```

4. **Lançar o Modelo**

   Utilize o comando `ros2 launch` para iniciar o `robot_state_publisher` com o arquivo de lançamento especificado:

   ```bash
   ros2 launch senai_models model_rsp.launch.py model:=nome_do_modelo
   ```

## Arquivos de Lançamento

O arquivo `model_rsp.launch.py` está configurado para carregar o modelo URDF e iniciar o `robot_state_publisher`. Abaixo está um exemplo simplificado de como o arquivo de lançamento é estruturado:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': '<path_para_seu_arquivo_urdf>'}]
        )
    ])
```

No arquivo real, o `<path_para_seu_arquivo_urdf>` deve ser substituído pelo caminho para o seu arquivo URDF específico.

## Contribuição

Contribuições são bem-vindas! Sinta-se à vontade para abrir issues e pull requests para melhorias, correções de bugs ou novos recursos.

## Licença

Este projeto está licenciado sob a Licença MIT - veja o arquivo [LICENSE](LICENSE) para mais detalhes.

---

Siga estas instruções para iniciar e utilizar os modelos de robô fornecidos neste repositório. Caso tenha dúvidas ou problemas, não hesite em abrir uma issue no GitHub.