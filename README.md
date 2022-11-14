# MODELING-A-HYBRID-SYSTEM-FOR-AN-AUTOMATED-GUIDED-VEHICLE---AGV
This project aims to model a hybrid system in the simulink of an AGV to follow a given trajectory.

1.PROCEDIMENTO EXPERIMENTAL

Inicialmente, montou-se o esquemático no stateflow com a máquina de estado, junto do bloco Robot Vizualizer da toolbox mobile robotics, para o desenho da trajetória apresentado no enunciado, usou-se um bloco waypoints também da toolbox mobile robotics para desenhar os pontos da trajetória na qual o AGV(“Automated Guided Vehicle”) deverá seguir, esses mesmos pontos usados para exibição 2D foram utilizados como pontos de referência na função do cálculo do erro no stateflow. É possível analisar a organização da modelagem na Figura 01.

                                          Figura 01 – Modelagem do AGV no simulink

![image](https://user-images.githubusercontent.com/102437900/201552115-a74f2d39-ab1e-406d-a18f-07e3c6672c3e.png)

                                                    Fonte: Próprio Autor
Posteriormente, povoou-se o bloco waypoints com as coordenadas da trajetória apresentada no enunciado do trabalho, que pode ser visto na Figura 02.
                                       Figura 02 – Trajetória apresentada no enunciado do problema
                                       ![image](https://user-images.githubusercontent.com/102437900/201552163-29df73dd-c011-4d8a-bf75-715d98e1186a.png)

 
                                                      Fonte: Descrição do problema
Na figura 03, pode visualizar a disposição dos pontos, referente a trajetória já devidamente organizados no simulink, para a movimentação do AGV.
                                      Figura 03 – Trajetória que será realizada pelo AGV
                                      ![image](https://user-images.githubusercontent.com/102437900/201552191-52f5dfc4-1bf0-4c35-b69b-94b4dd4e61b5.png)

 
                                                  Fonte: Próprio Autor

Por conseguinte, em posse das coordenadas da trajetória, montou-se a dinâmica hibrida do sistema, ou seja, montou-se a máquina de estado, contendo quatro estados: Stop, Straight, Rigth e Left, assim como todas as transições, baseadas nas entradas Start e Stop, além disso pode-se observar o bloco correspondente a função Matlab utilizada para o cálculo do erro de posição do robô, a organização das máquinas de estados pode ser analisada na Figura 04.

                                      Figura 04 – Organização da máquina de estados da dinâmica hibrida
                                      ![image](https://user-images.githubusercontent.com/102437900/201552225-9c892018-1f9e-46f6-b93e-4f09dd80fdde.png)

 
                                                                  Fonte Próprio Autor

Como pode ser analisado na Figura 04, foi utilizado, para realizar  as comparações ponto a ponto, ou seja, o ponto atual do robô no mapa com os valores definidos como a trajetória ideal um contador que é passado como parâmetro para a função do calculo de erro, cada vez que a velocidade do AGV é alterada em cada estado, espera-se que ele tenha um movimento, para isso, em cada estado e em cada transição existe um contador idx  que correspondo a quantos pontos o robô percorreu até o presente momento, diante disso, esse valor de idx ao ser passado como parâmetro para a função, indica dentro dela qual será a coordenada no índice idx que será comparado com o valor de x e y atual. Além de x, y e idx, theta também é passado como parâmetro da função pois desempenha um importante papel no cálculo do erro. Portanto, para percorrer todos os pontos da trajetória, uma variável é incrementada a cada movimento do robô, obviamente só não é incrementada no estado Stop por conta do robô está em inercia, entretanto nos demais estados há transições padrão para realizar a incrementação de idx até que o número total de pontos do mapa numPoints seja alcançado, ficando assim no estado Stop.Uma vez definidos todos os estados e transições, analisa-se na Figura 05 a definição de variáveis utilizadas no sistema.

                                        Figura 05 – Definição das variáveis utilizadas no sistema AGV
                                        ![image](https://user-images.githubusercontent.com/102437900/201552261-9cf325ba-0844-475d-9ce3-be1c5a7b776c.png)
                                                            Fonte : Próprio Autor
As variáveis x, y e theta foram definidas como variáveis locais e contínuas dado que são derivadas dentro do sistema. As variáveis x_saida , y_saida e theta_saida são variáveis de saída e stop e start variáveis de entrada, o restante das variáveis são locais com exceção da variável a que representa a velocidade máxima na qual o AGV se move. numPoints representa a quantidade de pontos utilizados na trajetória no caso foram 128 pontos. x0, y0 e theta0 são as condições iniciais de posição e orientação do robô enquanto e1 representa a faixa aceitável de erro, e e2 representa a faixa na qual as mudanças de orientação, ou seja, de estados do AGV começam a ocorrer. 
Dado todas as informações a Priore, pode-se apresentar a função para o cálculo do erro. A ideia do erro se dá por um desvio sofrido pelo robô da trajetória de referência, baseado no valor desse erro, o veículo deve ser capaz de que se maior ou menor que e2, alternar seu trajeto ou para esquerda ou para direita, caso o erro se concentre na faixa e1, o AGV pode seguir sua trajetória em linha reta, o que significa que o robô está próximo a pista.

Como dito no enunciado do problema, as coordenadas do veículo no instante t em relação ao referencial de coordenadas global são x(t), e(t) e ϴ(t). Além disso, foi adotado uma convenção na qual e(t)<0  siginifica que o AGV está à direita na pista, e e(t)>0 significa que o robô está à esquerda da pista. Diante dessas considerações levou-se em conta a distância entre dois pontos no modelo cartesiano para mensurar o erro, entretanto sabendo que o cálculo da distância entre dois pontos retorna um valor sempre positivo como ilustra a Eq.01
                                                 D=√((x_(b-) x_a )^2+(y_(b-) y_a )^2 )		              Eq.01
Onde, D é a medida da distância entre um ponto A de um Ponto B. A Eq.01 foi usada para o calculo do erro, entretanto, sozinha, a equação da distancia entre dois pontos nada diz sobre se o ponto a ser analisado está a direita ou a esquerda do ponto de referencia da trajetória, em combinação ao calculo da distancia entre dois pontos, utilizou-se uma técnica baseada na angulação do robô para definir o sinal do erro, se positivo porque está a esquerda ou negativo porque está a direita, antes dessa análise, pose-se observar na Figura 06 o código da função implementada.
                                              Figura 06 – Cálculo do erro de posicionamento do AGV
                                              ![image](https://user-images.githubusercontent.com/102437900/201552293-a2970e7d-ac83-43a9-9519-d4ef1bd729b6.png)
                                                                Fonte: Próprio Autor

Na variável pose1 é armazenada a diferença entre o ponto atual e o ponto de referência em x, enquanto na variável pose2 é dada pela diferença dos pontos em y. Na função pode-se analisar o uso do índice calculado nas transições da máquina de estado e usados para comparar os pontos atuais do AGV com o ponto de referência definido na matriz u.
Como previamente descrito, utilizou-se a angulação do AGV para determinar o sinal do erro conforme as especificações do problema, primeiramente faz-se necessário definir qual a angulação atual do AGV no mapa, para isso, basta realizar o cálculo da inclinação de uma reta cortando o ponto de referência e o ponto atual do robô. Na Figura 07 apresenta uma ideia visual desse cálculo.

                                                            Figura 07 – Ângulo de Inclinação
                                                            ![image](https://user-images.githubusercontent.com/102437900/201552329-339aedfa-4dae-4b49-b9af-c9c616d5861a.png)

 
                                                      Fonte : https://slideplayer.com.br/slide/334100/

Para análise pode-se supor que o ponto P na Figura 07 representa o ponto atual do AGV e o ponto Q representa a coordenada de referência, isolando α, tem-se a Eq.02

                                                       α=arctg (〖(y〗_Q-y_P))/(〖(x〗_Q-x_P))    	                                     Eq.02
Com a Eq.02 tem-se o ângulo de referência da posição dos pontos de referência e posição atual em relação a x e y. Na função usou-se uma função do matlab atan2() para o cálculo da tangente inversa, essa função difere da função atan() por conta de os resultados retornados por atan2()  pertencem ao intervalo fechado [-π, π] como foi solicitado no enunciado do problema, enquanto a função atan() retorna valores restritos ao intervalo [-π/2, π/2]. Por outro lado tem-se a ângulo atual do robô representado por theta na função, na qual é uma variável contínua no modelo e passada por parâmetro para a função fct, ora, se tem-se o ângulo formado pela reta que passa pelos pontos de referencia e o atual, e tem-se o ângulo atual do robô ‘theta’ então pode-se definir a posição do AGV nesse sentido, realizando a diferença entre esses dois valores calculados, ou seja, a diferença entre os ângulos theta, e o ângulo	de referência, para isso foi utilizado a função angdiff() do matlab que realiza o calculo da diferença entre dois ângulos. Na figura 08 é apresentado um esquemático exemplar do funcionamento dessa função para indicar a posição do AGV.

                                            Figura 08 -  Funcionamento para identificara a orientação do AGV
                                            ![image](https://user-images.githubusercontent.com/102437900/201552361-36872aec-121e-409e-97e4-d5e8119e1e5c.png)

 
                                                                  Fonte: Próprio Autor

No exemplo da figura 08 o ângulo de referência calculado através da Eq.02 retorna π/4 enquanto o robô está seguindo em linha reta, ou seja, theta = 0 é o ângulo atual do robô, ao usar a função angdiff() com esses valores, obtém-se que a diferença é o próprio π/4 o que significa que esse valor é maior que 0, ou seja positivo, o que garante dizer que o AGV está a direita, na função desenvolvida é usado estruturas condicionais para alterar o sinal do erro dado pela distancia entre dois pontos, se o valor da diferença é negativo, significa dizer que o AGV está a esquerda, logo não preciso alterar o sinal do erro, se essa condição não for satisfeita, significa dizer que a diferença é positiva e o AGV está a direita, e então o valor do erro é multiplicado por -1 para que o erro seja negativo como solicitado no enunciado do problema. Uma análise mais matemática do sinal do erro pode ser analisada na Eq.03.
                                                               ϴ+angdiff(α,ω)< ϴ	                          Eq.03
Ou seja, se a diferença entre os ângulos for somada ao ângulo atual e essa soma resultar em um valor inferior ao ângulo atual, significa que a diferença foi negativa, logo a posição atual do robô é à esquerda, caso contrário onde a diferença seja positiva, o AGV estará à direita do ponto referencial. De uma forma geral :

                           angdiff(α,ω)< 0        -    AGV está à esquerda do ponto referencial
                           angdiff(α,ω)> 0        -    AGV está à direita do ponto referencial

Dada toda a abordagem anterior, o AGV seguiu sua trajetória respeitando todos os critérios previamente expostos. É importante ressaltar que o robô se move sempre a uma velocidade máxima de 16km/h, dito isso notou-se na modelagem problemas envolvendo a escala dos pontos de coordenadas utilizadas, no sentido de que se aumenta-se a velocidade do robô ele ultrapassaria vários pontos e a análise com o ponto de coordenada referencial seria com uma coordenada errada, ou seja muito a frente da ideal, e isso o robô faria uma trajetória errônea, da mesma forma se a velocidade fosse muito baixa, ele iria pegar diversos pontos entre dois pontos de referência e iria realizar uma comparação, ou seja, o calculo do erro seria dado por índices que não correspondem ao que seria ideal, no caso ideal seria comparar ponto a ponto no mapa com os pontos de coordenas de referência, para isso poderia encontrar uma velocidade adequada que possibilitasse analisar adequadamente índice a índice, mas para essa resolução foi feita uma adaptação da escala de tal forma que a velocidade ideal seria 16km/h que é a velocidade solicitada no enunciado, adaptando a escala para essa velocidade, foi possível percorrer o mapa de pontos e analisa-los adequadamente com seu correspondente na matriz de pontos de referência.


2.RESULTADOS

Na figura 09 pode-se analisar a trajetória realizada pelo AGV com as marcações da trajetória de referência, na figura 10 pode-se analisar separadamente a trajetória do robô.

                                Figura 09 – Trajetória realizada pelo AGV com a marcação das coordenadas de referência
                                ![image](https://user-images.githubusercontent.com/102437900/201552399-3cde7ea3-12ff-42a9-a421-4394a55d635e.png)

 
                                                                      Fonte: Próprio Autor

                                                        Figura 10  - Trajetória realizada pelo AGV
                                                        ![image](https://user-images.githubusercontent.com/102437900/201552428-72e896cc-3bae-4be1-8efb-107e203b2f3a.png)

 
                                                                      Fonte: Próprio Autor

Pode-se analisar pela Figura 09 que a trajetória corresponde um contorno aproximadamente igual ao trajeto de referência, tendo um erro bem pequeno apresentando um resultado bastante satisfatório, vale ressaltar que as métricas e1 foi de 0.1 e e2 foi de 0.5 como pode-se analisar na Figura 05.
Na figura 11 pode-se observar quando após realizar uma determinada trajetória e a entrada start posteriormente for acionada para um valor lógico false.


                                                Figura 11 – Trajetória interrompida após alguns instantes de tempo
                                                ![image](https://user-images.githubusercontent.com/102437900/201552461-731724cc-104e-4cc9-afec-7d80d3686d78.png)
                                                                      Fonte: Próprio Autor

Dessa forma, diante de toda abordagem, a modelagem de um sistema híbrido para controle de um veículo seguidor de linha AGV mostrou-se bastante satisfatório executando perfeitamente os requisitos predefinidos no enunciado e apresentando uma trajetória bem fidedigna a trajetória de referência.
