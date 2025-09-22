## Navegação Reativa com TurtleBot3 (ROS 2 + Gazebo Classic)

Este projeto demonstra navegação reativa com o TurtleBot3 (modelo waffle) no Gazebo Classic, usando ROS 2 Humble. Implementa dois algoritmos clássicos:
- Bug2: segue para o objetivo, contorna obstáculos e retorna à linha de início‑objetivo (M‑line) quando seguro e mais próximo do goal que no ponto de contato.
- TangentBug: seleciona candidatos por bordas da leitura do laser e decide o melhor avanço com base em um custo heurístico.

O mundo `bug_world.world` contém obstáculos estáticos e dois “spots” de luz que definem início e objetivo:
- START: `user_spot_light_0`
- GOAL:  `user_spot_light_1`

Como rodar (Docker Compose)
- Requisitos: Docker + Docker Compose, X11 habilitado no host Linux.
- No host: `xhost +local:`
- Suba o ambiente:
  - `docker-compose up -d --build`
- Entre no container e rode:
  - `docker exec -it tb3_nav bash`
  - `source /opt/ros/humble/setup.bash`
  - `colcon build --packages-select tb3_bug_nav_classic --symlink-install`
  - `source /root/ws/install/setup.bash`

Lançamentos
- Bug2: `ros2 launch tb3_bug_nav_classic bug2_classic.launch.py world:=/root/ws/src/tb3_bug_nav_classic/worlds/bug_world.world gui:=true`
- Tangent‑Bug:   `ros2 launch tb3_bug_nav_classic tangent_classic.launch.py world:=/root/ws/src/tb3_bug_nav_classic/worlds/bug_world.world gui:=true`

Arquitetura
```
.
├─ Dockerfile                  
├─ docker-compose.yml          
├─ entrypoint.sh               
├─ README.md                   
└─ ws/
   └─ src/
      └─ tb3_bug_nav_classic/
         ├─ package.xml        
         ├─ setup.py           
         ├─ setup.cfg          
         ├─ resource/
         │  └─ tb3_bug_nav_classic   
         ├─ worlds/
         │  └─ bug_world.world  # mundo Gazebo 
         ├─ launch/
         │  ├─ bug2_classic.launch.py     # Bug2 + Gazebo 
         │  ├─ tangent_classic.launch.py  # TangentBug + Gazebo 
         └─ tb3_bug_nav_classic/
            ├─ __init__.py
            ├─ utils.py           
            ├─ bug2_node.py        # Bug2 (to-goal, follow-boundary, leave-to-goal)
            └─ tangent_bug_node.py # TangentBug (to-goal, follow-boundary, leave-to-point)
