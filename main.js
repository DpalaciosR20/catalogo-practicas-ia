        // Lista de prácticas como objetos
        const practicas = [
            {
                archivo: "P1",
                titulo: "Práctica 1",
                nombre: "Agentes Inteligentes",
                descripcion: "Robot aspiradora: Los sensores detectan obstáculos (percepción), los datos se procesan para crear un mapa del entorno, el sistema toma decisiones sobre la dirección a seguir (razonamiento), y los motores actúan (actuadores). Si el robot queda atascado, puede aprender a evitar esa área en el futuro (aprendizaje).",
                codigo:
                    `
clc;
clear;

% Definición del entorno (5x5 grid)
gridSize = 5;
grid = zeros(gridSize); % 0: libre, 1: obstáculo

% Posicionar obstáculos
grid(4, 3) = 1;
grid(2, 2) = 1;


% Definir posición inicial del agente y objetivo
agentPos = [1, 1]; % Inicio en la esquina superior izquierda
goalPos = [5, 5]; % Meta en la esquina inferior derecha

% Función para mostrar la cuadrícula
function showGrid(grid, agentPos, goalPos)
    clf;
    imagesc(grid);
    colormap(gray);
    hold on;
    plot(agentPos(2), agentPos(1), 'bo', 'MarkerSize', 15, 'LineWidth', 3); % Agente
    plot(goalPos(2), goalPos(1), 'ro', 'MarkerSize', 15, 'LineWidth', 3); % Objetivo
    title('Agente navegando en la cuadrícula');
    drawnow;
end

% Mostrar la cuadrícula inicial
showGrid(grid, agentPos, goalPos);

% Definir las posibles acciones (movimientos)
actions = [0, 1; 0, -1; 1, 0; -1, 0]; % Derecha, Izquierda, Abajo, Arriba

% Mover al agente hasta alcanzar la meta o quedar atrapado
while ~isequal(agentPos, goalPos)
    % Mostrar la cuadrícula
    showGrid(grid, agentPos, goalPos);
    
    % Leer los sensores (percibir el entorno)
    % Checar si los movimientos posibles están bloqueados por obstáculos o límites del grid
    validMoves = [];
    for i = 1:size(actions, 1)
        newPos = agentPos + actions(i, :);
        if newPos(1) >= 1 && newPos(1) <= gridSize && newPos(2) >= 1 && newPos(2) <= gridSize
            if grid(newPos(1), newPos(2)) == 0 % No hay obstáculo
                validMoves = [validMoves; actions(i, :)];
            end
        end
    end

    % Decisión (elegir un movimiento aleatorio válido)
    if isempty(validMoves)
        disp('El agente está atrapado.');
        break;
    else
        % Tomar una decisión aleatoria basada en los movimientos válidos
        move = validMoves(randi(size(validMoves, 1)), :);
        agentPos = agentPos + move; % Actualizar la posición del agente
    end

    pause(0.5); % Pausa para visualizar el movimiento
end

% Verificar si el agente llegó a la meta
if isequal(agentPos, goalPos)
    disp('El agente ha llegado a su objetivo.');
else
    disp('El agente no pudo llegar a su objetivo.');
end`
            },
            {
                archivo: "P2",
                titulo: "Práctica 2: BFS",
                nombre: "Algoritmos de búsqueda no informados",
                descripcion: "Ejemplo de búsqueda en anchura en MATLAB",
                codigo: `
% Definimos el grafo como una matriz de adyacencia
% Ejemplo de grafo:
% Nodo 1 está conectado a 2 y 3
% Nodo 2 está conectado a 4
% Nodo 3 está conectado a 4 y 5
% Nodo 4 está conectado a 6
% Nodo 5 está conectado a 6
% Nodo 6 no tiene más conexiones
adjMatrix = [0 1 1 0 0 0; % Nodo 1
            0 0 0 1 0 0; % Nodo 2
            0 0 0 1 1 0; % Nodo 3
            0 0 0 0 0 1; % Nodo 4
            0 0 0 0 0 1; % Nodo 5
            0 0 0 0 0 0]; % Nodo 6

function bfs(adjMatrix, startNode)
    % Número de nodos en el grafo
    numNodes = size(adjMatrix, 1);
    % Inicializar una cola para nodos por visitar
    queue = [startNode];
    % Vector para marcar los nodos visitados
    visited = false(1, numNodes);
    
    % Marcar el nodo inicial como visitado
    visited(startNode) = true;
    % Iterar mientras haya nodos en la cola
    while ~isempty(queue)
        % Extraer el primer nodo de la cola
        currentNode = queue(1);
        queue(1) = [];
        % Mostrar el nodo actual
        fprintf('Visitando nodo %d\n', currentNode);
        % Revisar los nodos adyacentes
        for neighbor = 1:numNodes
            if adjMatrix(currentNode, neighbor) == 1 && ~visited(neighbor)
                % Si hay una conexión y el nodo no ha sido visitado, agregarlo a la cola
                queue = [queue, neighbor];
                % Marcar el vecino como visitado
                visited(neighbor) = true;
            end
        end
    end
end

startNode = 1;
bfs(adjMatrix, startNode);`
            },
            {
                archivo: "P2",
                titulo: "Práctica 2: DFS",
                nombre: "Algoritmos de búsqueda no informados",
                descripcion: "Ejemplo de búsqueda en profundidad con Matlab",
                codigo: `
% Definimos el grafo como una matriz de adyacencia
% Ejemplo de grafo:
% Nodo 1 está conectado a 2 y 3
% Nodo 2 está conectado a 4
% Nodo 3 está conectado a 4 y 5
% Nodo 4 está conectado a 6
% Nodo 5 está conectado a 6
% Nodo 6 no tiene más conexiones
adjMatrix = [0 1 1 0 0 0; % Nodo 1
            0 0 0 1 0 0; % Nodo 2
            0 0 0 1 1 0; % Nodo 3
            0 0 0 0 0 1; % Nodo 4
            0 0 0 0 0 1; % Nodo 5
            0 0 0 0 0 0]; % Nodo 6

function dfs(adjMatrix, currentNode, visited)
    % Número de nodos en el grafo
    numNodes = size(adjMatrix, 1);
    % Marcar el nodo actual como visitado
    visited(currentNode) = true;
    % Mostrar el nodo actual
    fprintf('Visitando nodo %d\n', currentNode);
    % Recorrer los vecinos no visitados del nodo actual
    for neighbor = 1:numNodes
        if adjMatrix(currentNode, neighbor) == 1 && ~visited(neighbor)
            % Si el vecino no ha sido visitado, hacer la llamada recursiva
            dfs(adjMatrix, neighbor, visited);
        end
    end
end

% Definir el nodo de inicio
startNode = 1;
% Inicializar el vector de nodos visitados
visited = false(1, size(adjMatrix, 1));
% Llamar a la función de búsqueda en profundidad
dfs(adjMatrix, startNode, visited);
`
            },
            {
                archivo: "P3",
                titulo: "Práctica 3",
                nombre: "Búsqueda informada",
                descripcion: "Ejemplo de Algoritmo A* en MATLAB",
                codigo: `
% Definir la matriz de adyacencia (grafo)
nodos = 6; % Número de nodos
A = [0 1 4 inf inf inf;
     1 0 2 6 inf inf;
     4 2 0 3 3 inf;
     inf 6 3 0 2 5;
     inf inf 3 2 0 2;
     inf inf inf 5 2 0];
% Definir los puntos de los nodos (para la heurística, puede ser una distancia euclidiana)
coordenadas = [0 0;
               1 1;
               2 2;
               3 2;
               4 3;
               5 3];
% Definir el nodo de inicio y objetivo
inicio = 1;
objetivo = 6;

% Función heurística (distancia euclidiana a la meta)
function h = heuristica(nodo, objetivo, coordenadas)
    h = norm(coordenadas(nodo,:) - coordenadas(objetivo,:));
end

% Algoritmo A*
function [camino, costo_total] = A_star(A, inicio, objetivo, coordenadas)
    nodos = size(A, 1);
    % Inicializar costos y listas
    g = inf(1, nodos); % Costo desde el inicio a cada nodo
    f = inf(1, nodos); % Costo estimado total (g + h)
    g(inicio) = 0;
    f(inicio) = heuristica(inicio, objetivo, coordenadas);
    % Lista abierta (nodos por explorar) y cerrada (ya explorados)
    abierta = [inicio];
    cerrada = [];
    % Predecesores (para construir el camino final)
    predecesor = zeros(1, nodos);
    while ~isempty(abierta)
        % Encontrar el nodo con menor f en la lista abierta
        [~, idx] = min(f(abierta));
        actual = abierta(idx);
        % Si llegamos al nodo objetivo, construimos el camino
        if actual == objetivo
            camino = [];
            while actual ~= 0
                camino = [actual, camino]; %#ok<AGROW>
                actual = predecesor(actual);
            end
            costo_total = g(objetivo);
            return;
        end
        
        % Mover el nodo actual de la lista abierta a la cerrada
        abierta(idx) = [];
        cerrada = [cerrada, actual];

        % Examinar los vecinos del nodo actual
        for vecino = 1:nodos
            if A(actual, vecino) ~= inf && ~ismember(vecino, cerrada)
                tentativo_g = g(actual) + A(actual, vecino);
                if ~ismember(vecino, abierta)
                    abierta = [abierta, vecino];
                end
                if tentativo_g < g(vecino)
                    % Actualizar el costo g y f
                    predecesor(vecino) = actual;
                    g(vecino) = tentativo_g;
                    f(vecino) = g(vecino) + heuristica(vecino, objetivo, coordenadas);
                end
            end
        end
    end
    % Si no se encuentra camino
    camino = [];
    costo_total = inf;
    disp('No se encontró camino');
end

[camino, costo_total] = A_star(A, inicio, objetivo, coordenadas);
disp('Camino encontrado:');
disp(camino);
disp('Costo total:');
disp(costo_total);
`
            },
            {
                archivo: "P4",
                titulo: "Práctica 4: 3 en raya",
                nombre: "Búsqueda Adversaria",
                descripcion: "Algoritmo Minimax en MATLAB para tres en raya",
                codigo: `
% Minimax algorithm for Tic-Tac-Toe (Tres en Raya)
function bestMove = minimax_example()
    % Inicializa el tablero (0 = vacío, 1 = X, -1 = O)
    board = [0 0 0; 0 0 0; 0 0 0];
    
    % Llama a la función minimax para encontrar el mejor movimiento
    [~, bestMove] = minimax(board, 1);
end

% Función Minimax
% board: el estado actual del tablero
% player: 1 (MAX, jugador X) o -1 (MIN, jugador O)
function [score, move] = minimax(board, player)
    % Revisa si hay un ganador o si es empate
    winner = check_winner(board);
    
    if winner ~= 0
        % Retorna la evaluación: 1 si X gana, -1 si O gana, 0 si es empate
        score = winner;
        move = [];
        return;
    elseif isempty(find(board == 0, 1))
        % Si no quedan movimientos posibles, es empate
        score = 0;
        move = [];
        return;
    end

    % Inicialización
    bestScore = -inf * ,;
    bestMove = [];

    % Bucle para probar cada posible movimiento
    for i = 1:3
        for j = 1:3
            if board(i, j) == 0
                % Simula el movimiento
                board(i, j) = player;
                
                % Evalúa el tablero después de ese movimiento
                [nextScore, ~] = minimax(board, -player);
                
                % Deshace el movimiento
                board(i, j) = 0;
                
                % Actualiza el mejor movimiento
                if (player == 1 && nextScore > bestScore) || (player == -1 && ...
                    nextScore < bestScore)
                    bestScore = nextScore;
                    bestMove = [i, j];
                end
            end
        end
    end

    % Retorna la mejor puntuación y el mejor movimiento
    score = bestScore;
    move = bestMove;
end

% Función para revisar si hay un ganador
function winner = check_winner(board)
    % Comprobación de filas, columnas y diagonales
    for i = 1:3
        if abs(sum(board(i, :))) == 3 % Revisa filas
            winner = sign(sum(board(i, :)));
            return;
        elseif abs(sum(board(:, i))) == 3 % Revisa columnas
            winner = sign(sum(board(:, i)));
            return;
        end
    end
    % Revisa diagonales
    if abs(sum(diag(board))) == 3
        winner = sign(sum(diag(board)));
        return;
    elseif abs(sum(diag(flipud(board)))) == 3
        winner = sign(sum(diag(flipud(board))));
        return;
    end

    % Si no hay ganador
    winner = 0;
end
`
            },
            {
                archivo: "P4",
                titulo: "Práctica 4: Dilema del prisionero",
                nombre: "Búsqueda Adversaria",
                descripcion: "Implementación del Dilema del Prisionero en MATLAB",
                codigo: `
% Dilema del Prisionero en MATLAB
% Definir los pagos para las posibles acciones
% Matriz de pagos: [pago_A, pago_B]
payoffs = [
    -3, -3; % Ambos cooperan
    0, -10; % A traiciona, B coopera
    -10, 0; % A coopera, B traiciona
    -5, -5 % Ambos traicionan
];

% Acciones posibles
actions = {'Cooperar', 'Traicionar'};

% Elección de los jugadores (1 = Cooperar, 2 = Traicionar)
% Puedes cambiar estas elecciones para ver distintos resultados
choice_A = 1; % Elección del prisionero A
choice_B = 1; % Elección del prisionero B

% Determinar el pago según las elecciones
if choice_A == 1 && choice_B == 1
    outcome = payoffs(1, :); % Ambos cooperan
elseif choice_A == 2 && choice_B == 1
    outcome = payoffs(2, :); % A traiciona, B coopera
elseif choice_A == 1 && choice_B == 2
    outcome = payoffs(3, :); % A coopera, B traiciona
elseif choice_A == 2 && choice_B == 2
    outcome = payoffs(4, :); % Ambos traicionan
end

% Mostrar los resultados
disp(['Prisionero A elige: ', actions{choice_A}]);
disp(['Prisionero B elige: ', actions{choice_B}]);
disp(' ');
disp('Resultado (Años de cárcel para A, Años de cárcel para B): ');
disp(['(',num2str(outcome),')']);
`
            },
            {
                archivo: "P5",
                titulo: "Práctica 5",
                nombre: "Lógica de primer orden",
                descripcion: `
Ejercicio propuesto:
Una empresa de distribución de mercancías tiene varios centros de distribución (CDs) y
clientes ubicados en diferentes ciudades. El objetivo es determinar si es posible entregar
todos los pedidos en un tiempo máximo determinado (por ejemplo, 48 horas) desde algún
centro de distribución cercano. Si no es posible, se deben identificar los clientes que no
pueden ser atendidos dentro del límite de tiempo.`,
                codigo: `
classdef EP < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure              matlab.ui.Figure
        HelpButton            matlab.ui.control.Button
        CentrosSpinner        matlab.ui.control.Spinner
        CentrosSpinnerLabel   matlab.ui.control.Label
        ClientesSpinner       matlab.ui.control.Spinner
        ClientesSpinnerLabel  matlab.ui.control.Label
        ResultadosTextArea    matlab.ui.control.TextArea
        ResultadosLabel       matlab.ui.control.Label
        CalcularButton        matlab.ui.control.Button
        DistanciasTable       matlab.ui.control.Table
    end



    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Configurar la tabla para entrada de distancias
            app.DistanciasTable.ColumnName = {'Cliente', 'Centro'};
            app.DistanciasTable.RowName = {'Cliente'};
        end

        % Button pushed function: CalcularButton
        function CalcularButtonPushed(app, event)
            % Obtener el número de cliente
            numClientes = app.ClientesSpinner.Value;
            
            % Obtener las distancias de la tabla
            distancias = app.DistanciasTable.Data;

            % Definir la velocidad promedio de entrega
            velocidad_promedio = 2; % km/h
            tmax = 48; % tiempo máximo permitido en horas

            % Calcular el tiempo de entrega
            T = distancias / velocidad_promedio;

            % Verificar si cada cliente puede ser atendido por al menos un centro dentro del tiempo permitido
            % ∀x ∃y T(x,y)≤tmax.
            cumple_tiempo = arrayfun(@(i) any(T(i, :) <= tmax), 1:numClientes);

            % Mostrar resultados en el área de texto
            if all(cumple_tiempo)
                app.ResultadosTextArea.Value = 'Todos los clientes pueden ser atendidos dentro del tiempo permitido.';
            else
                clientesNoAtendidos = find(~cumple_tiempo);
                listaClientes = sprintf('Cliente %d, ', clientesNoAtendidos);
                app.ResultadosTextArea.Value = ['Clientes no atendidos: ', listaClientes];
            end
        end

        % Value changed function: CentrosSpinner
        function CentrosSpinnerChanged(app, event)
            numClientes = app.ClientesSpinner.Value;
            numCentros = app.CentrosSpinner.Value;
            % Ajustar el tamaño de la tabla
            app.DistanciasTable.Data = zeros(numClientes, numCentros); % Inicializa la tabla con ceros
            app.DistanciasTable.ColumnName = arrayfun(@(c) sprintf('Centro %d', c), 1:numCentros, 'UniformOutput', false);
            app.DistanciasTable.RowName = arrayfun(@(c) sprintf('Cliente %d', c), 1:numClientes, 'UniformOutput', false);
        end

        % Value changed function: ClientesSpinner
        function ClientesSpinnerChanged(app, event)
            numClientes = app.ClientesSpinner.Value;
            numCentros = app.CentrosSpinner.Value;
            % Ajustar el tamaño de la tabla
            app.DistanciasTable.Data = zeros(numClientes, numCentros); % Inicializa la tabla con ceros
            app.DistanciasTable.ColumnName = arrayfun(@(c) sprintf('Centro %d', c), 1:numCentros, 'UniformOutput', false);
            app.DistanciasTable.RowName = arrayfun(@(c) sprintf('Cliente %d', c), 1:numClientes, 'UniformOutput', false);
        end

        % Callback function
        function Ayuda(app, event)
            
        end

        % Callback function
        function AyudaButton(app, event)
            
        end

        % Button pushed function: HelpButton
        function AyudaButtonPushed(app, event)
            % Mostrar una ventana emergente con información sobre cómo funciona el programa
            uialert(app.UIFigure, ...
                ['Este programa calcula si todos los clientes pueden ser atendidos dentro del tiempo permitido. ' ...
                 'Primero, debes ingresar el número de clientes y centros de distribución usando los controles "Clientes" y "Centros". ' ...
                 'Luego, ingresa las distancias entre cada cliente y cada centro en la tabla proporcionada. ' ...
                 'Finalmente, presiona el botón "Calcular" para obtener los resultados en el área de texto. ' ...
                 'Si algún cliente no puede ser atendido a tiempo, se listará en los resultados.'], ...
                'Ayuda', 'Icon', 'info');
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 639 482];
            app.UIFigure.Name = 'MATLAB App';

            % Create DistanciasTable
            app.DistanciasTable = uitable(app.UIFigure);
            app.DistanciasTable.ColumnName = {'Cliente'; 'Centro'};
            app.DistanciasTable.RowName = {};
            app.DistanciasTable.ColumnEditable = true;
            app.DistanciasTable.Position = [42 190 302 185];

            % Create CalcularButton
            app.CalcularButton = uibutton(app.UIFigure, 'push');
            app.CalcularButton.ButtonPushedFcn = createCallbackFcn(app, @CalcularButtonPushed, true);
            app.CalcularButton.Position = [400 353 100 22];
            app.CalcularButton.Text = 'Calcular';

            % Create ResultadosLabel
            app.ResultadosLabel = uilabel(app.UIFigure);
            app.ResultadosLabel.HorizontalAlignment = 'right';
            app.ResultadosLabel.Position = [40 141 68 22];
            app.ResultadosLabel.Text = 'Resultados:';

            % Create ResultadosTextArea
            app.ResultadosTextArea = uitextarea(app.UIFigure);
            app.ResultadosTextArea.Position = [43 31 495 111];

            % Create ClientesSpinnerLabel
            app.ClientesSpinnerLabel = uilabel(app.UIFigure);
            app.ClientesSpinnerLabel.HorizontalAlignment = 'right';
            app.ClientesSpinnerLabel.Position = [44 425 48 22];
            app.ClientesSpinnerLabel.Text = 'Clientes';

            % Create ClientesSpinner
            app.ClientesSpinner = uispinner(app.UIFigure);
            app.ClientesSpinner.ValueChangedFcn = createCallbackFcn(app, @ClientesSpinnerChanged, true);
            app.ClientesSpinner.Position = [107 425 100 22];
            app.ClientesSpinner.Value = 1;

            % Create CentrosSpinnerLabel
            app.CentrosSpinnerLabel = uilabel(app.UIFigure);
            app.CentrosSpinnerLabel.HorizontalAlignment = 'right';
            app.CentrosSpinnerLabel.Position = [357 425 47 22];
            app.CentrosSpinnerLabel.Text = 'Centros';

            % Create CentrosSpinner
            app.CentrosSpinner = uispinner(app.UIFigure);
            app.CentrosSpinner.ValueChangedFcn = createCallbackFcn(app, @CentrosSpinnerChanged, true);
            app.CentrosSpinner.Position = [419 425 100 22];
            app.CentrosSpinner.Value = 1;

            % Create HelpButton
            app.HelpButton = uibutton(app.UIFigure, 'push');
            app.HelpButton.ButtonPushedFcn = createCallbackFcn(app, @AyudaButtonPushed, true);
            app.HelpButton.Position = [552 31 22 24];
            app.HelpButton.Text = '?';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = EP

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end
`
            },
            {
                archivo: "P6",
                titulo: "Práctica 6",
                nombre: "Razonamiento basado en reglas",
                descripcion: `Ejercicio propuesto
Desarrolle un programa con interfaz gráfica que determina la velocidad de giro de un
ventilador de tal manera que se mantenga la temperatura a 20º centígrados en un salón de
clase con capacidad máxima de 36 alumnos. La temperatura será siempre de 20º sin importar
el número de alumnos que haya en el aula.`,
                codigo: `
classdef ControlVentiladorApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                     matlab.ui.Figure
        Button                       matlab.ui.control.Button
        VelocidadVentiladorGauge     matlab.ui.control.LinearGauge
        VelocidaddelVentiladorLabel  matlab.ui.control.Label
        NumeroAlumnosSlider          matlab.ui.control.Slider
        NmerodeAlumnosLabel          matlab.ui.control.Label
    end

    properties (Access = private)
        sistema_fuzzy % Controlador difuso
    end
    
    methods (Access = private)
        function inicializarSistemaFuzzy(app)
            % Crear el sistema difuso
            app.sistema_fuzzy = mamfis('Name','Control Ventilador');

            % Agregar entrada de alumnos (0 a 36)
            app.sistema_fuzzy = addInput(app.sistema_fuzzy, [0 36], 'Name', 'Alumnos');
            app.sistema_fuzzy = addMF(app.sistema_fuzzy, 'Alumnos', 'trimf', [0 0 12], 'Name', 'Pocos');
            app.sistema_fuzzy = addMF(app.sistema_fuzzy, 'Alumnos', 'trimf', [10 18 26], 'Name', 'Moderados');
            app.sistema_fuzzy = addMF(app.sistema_fuzzy, 'Alumnos', 'trimf', [24 36 36], 'Name', 'Lleno');

            % Agregar salida de velocidad del ventilador (0 a 10)
            app.sistema_fuzzy = addOutput(app.sistema_fuzzy, [0 10], 'Name', 'Velocidad');
            app.sistema_fuzzy = addMF(app.sistema_fuzzy, 'Velocidad', 'trimf', [0 0 5], 'Name', 'Baja');
            app.sistema_fuzzy = addMF(app.sistema_fuzzy, 'Velocidad', 'trimf', [2.5 5 7.5], 'Name', 'Media');
            app.sistema_fuzzy = addMF(app.sistema_fuzzy, 'Velocidad', 'trimf', [5 10 10], 'Name', 'Alta');

            % Definir las reglas
            reglas = [
                "If Alumnos is Pocos then Velocidad is Baja"
                "If Alumnos is Moderados then Velocidad is Media"
                "If Alumnos is Lleno then Velocidad is Alta"
            ];
            app.sistema_fuzzy = addRule(app.sistema_fuzzy, reglas);

            % Visualizar el sistema difuso
            figure
            plotfis(app.sistema_fuzzy)

            % Evaluar el sistema difuso con diferentes numeros de alumnos
            numAlumnos = [5, 15, 25, 36]; % Ejemplos de entradas de temperatura
            for i = 1:length(numAlumnos)
                num = numAlumnos(i);
                velocidad = evalfis(app.sistema_fuzzy, num);
                fprintf('Para una cantidad de %d alumnos, la velocidad del ventilador es %.2f\n', num, velocidad);
            end
            
            % Visualizar las funciones de pertenencia
            figure
            subplot(2,1,1)
            plotmf(app.sistema_fuzzy, 'input', 1)
            title('Funciones de pertenencia del número de alumnos')
            subplot(2,1,2)
            plotmf(app.sistema_fuzzy, 'output', 1)
            title('Funciones de pertenencia de la Velocidad')
        end

        function actualizarVelocidad(app)
            % Obtener número de alumnos del deslizador
            numAlumnos = app.NumeroAlumnosSlider.Value;

            % Evaluar el sistema difuso
            velocidad = evalfis(app.sistema_fuzzy, numAlumnos);

            % Mostrar la velocidad en el indicador de velocímetro
            app.VelocidadVentiladorGauge.Value = velocidad;
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Inicializar el sistema difuso al iniciar la app
            inicializarSistemaFuzzy(app);
        end

        % Value changed function: NumeroAlumnosSlider
        function NumeroAlumnosSliderValueChanged(app, event)
            % Llamar a la función para actualizar la velocidad
            actualizarVelocidad(app);
        end

        % Button pushed function: Button
        function AyudaButtonPushed(app, event)
            % Mostrar una ventana emergente con información sobre cómo funciona el programa
            uialert(app.UIFigure, ...
                ['Este sistema ajusta la velocidad del ventilador para mantener la temperatura del salón en 20°C,' ...
                 'independientemente del número de alumnos presentes. La velocidad aumenta a medida que el' ...
                 'salón se llena, compensando el calor generado por los ocupantes. El control se basa en tres niveles' ...
                 'de ocupación: pocos, moderados y lleno, y ajusta automáticamente la velocidad del ventilador a' ...
                 'baja, media o alta según la cantidad de personas, asegurando un ambiente cómodo y estable.'], ...
                'Explicación del Sistema de Control de Ventilador:', 'Icon', 'info');
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 400 300];
            app.UIFigure.Name = 'Control de Ventilador';

            % Create NmerodeAlumnosLabel
            app.NmerodeAlumnosLabel = uilabel(app.UIFigure);
            app.NmerodeAlumnosLabel.HorizontalAlignment = 'right';
            app.NmerodeAlumnosLabel.Position = [31 253 114 22];
            app.NmerodeAlumnosLabel.Text = 'Número de Alumnos';

            % Create NumeroAlumnosSlider
            app.NumeroAlumnosSlider = uislider(app.UIFigure);
            app.NumeroAlumnosSlider.Limits = [0 36];
            app.NumeroAlumnosSlider.ValueChangedFcn = createCallbackFcn(app, @NumeroAlumnosSliderValueChanged, true);
            app.NumeroAlumnosSlider.Position = [160 262 204 3];

            % Create VelocidaddelVentiladorLabel
            app.VelocidaddelVentiladorLabel = uilabel(app.UIFigure);
            app.VelocidaddelVentiladorLabel.HorizontalAlignment = 'center';
            app.VelocidaddelVentiladorLabel.Position = [126 83 150 22];
            app.VelocidaddelVentiladorLabel.Text = 'Velocidad del Ventilador';

            % Create VelocidadVentiladorGauge
            app.VelocidadVentiladorGauge = uigauge(app.UIFigure, 'linear');
            app.VelocidadVentiladorGauge.Limits = [0 10];
            app.VelocidadVentiladorGauge.Position = [126 123 150 50];

            % Create Button
            app.Button = uibutton(app.UIFigure, 'push');
            app.Button.ButtonPushedFcn = createCallbackFcn(app, @AyudaButtonPushed, true);
            app.Button.BackgroundColor = [0.8471 0.9137 0.9412];
            app.Button.FontSize = 18;
            app.Button.FontWeight = 'bold';
            app.Button.Position = [355 15 26 28];
            app.Button.Text = '?';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = ControlVentiladorApp

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end
`
            },
            {
                archivo: "P7",
                titulo: "Práctica 7",
                nombre: "Modelos de llenado de ranuras",
                descripcion: `Ejercicio propuesto
Haga un sistema básico de llenado de ranuras que se use para una aplicación de reserva de
viajes. En este sistema, el usuario puede ingresar frases como "Reserva un vuelo de Madrid
a Londres el 10 de diciembre", y el modelo identificará las ranuras Origen, Destino y
Fecha.`,
                codigo: `
function sistema_reserva_gui
    % Crear la ventana principal
    fig = uifigure('Name', 'Sistema de Reserva', 'Position', [500, 200, 450, 450]);

    % Título principal
    uilabel(fig, ...
        'Text', 'Sistema de Reserva de Viajes', ...
        'FontSize', 16, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', ...
        'Position', [50, 400, 350, 40]);

    % Etiqueta de la entrada
    uilabel(fig, ...
        'Text', 'Ingrese su reserva:', ...
        'FontSize', 12, ...
        'HorizontalAlignment', 'left', ...
        'Position', [30, 350, 200, 30]);

    % Cuadro de entrada de texto
    entradaTexto = uitextarea(fig, ...
        'Position', [30, 310, 390, 40], ...
        'FontSize', 12);

    % Ejemplo de entrada
    uilabel(fig, ...
        'Text', 'Ejemplo: Reserva un vuelo de Madrid a Londres el 10 de diciembre', ...
        'FontSize', 10, ...
        'FontColor', [0.4, 0.4, 0.4], ...
        'HorizontalAlignment', 'left', ...
        'Position', [30, 280, 390, 30]);

    % Botón para procesar
    botonProcesar = uibutton(fig, ...
        'Text', 'Procesar', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.2, 0.6, 1], ...
        'FontColor', [1, 1, 1], ...
        'Position', [175, 230, 100, 40]);

    % Etiqueta para resultados
    uilabel(fig, ...
        'Text', 'Los detalles de la reserva son:', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Position', [30, 200, 390, 30]);

    % Etiqueta y resultado del origen
    salidaOrigen = uilabel(fig, ...
        'Text', '', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Position', [130, 160, 290, 30]);

    uilabel(fig, 'Text', 'Origen:', ...
        'FontSize', 12, ...
        'Position', [30, 160, 100, 30]);

    % Etiqueta y resultado del destino
    salidaDestino = uilabel(fig, ...
        'Text', '', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Position', [130, 120, 290, 30]);

    uilabel(fig, 'Text', 'Destino:', ...
        'FontSize', 12, ...
        'Position', [30, 120, 100, 30]);

    % Etiqueta y resultado de la fecha
    salidaFecha = uilabel(fig, ...
        'Text', '', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'Position', [130, 80, 290, 30]);

    uilabel(fig, 'Text', 'Fecha:', ...
        'FontSize', 12, ...
        'Position', [30, 80, 100, 30]);

    % Configurar el callback del botón
    botonProcesar.ButtonPushedFcn = @(~, ~) procesarReserva(entradaTexto, salidaOrigen, salidaDestino, salidaFecha);
end

function procesarReserva(entradaTexto, salidaOrigen, salidaDestino, salidaFecha)
    % Obtener la frase ingresada
    oracion = entradaTexto.Value{1}; % Obtener el texto de la entrada

    % Patrones para identificar las ranuras
    patron_origen = 'de\s+([A-Za-z\s]+?)\s+a'; % Captura el origen después de "de" y antes de "a"
    patron_destino = '\sa\s+([A-Za-z]+)'; % Captura solo el destino después de " a "
    patron_fecha = 'el\s+(\d{1,2}\s+de\s+[a-zA-Z]+)'; % Captura la fecha después de "el"

    % Buscar coincidencias usando expresiones regulares
    origen = regexp(oracion, patron_origen, 'tokens', 'once');
    destino = regexp(oracion, patron_destino, 'tokens', 'once');
    fecha = regexp(oracion, patron_fecha, 'tokens', 'once');

    % Validar resultados
    if isempty(origen)
        origen = {'No encontrado'};
    end

    if isempty(destino)
        destino = {'No encontrado'};
    end

    if isempty(fecha)
        fecha = {'No encontrado'};
    end

    % Mostrar los resultados en los componentes de la GUI
    salidaOrigen.Text = origen{1};
    salidaDestino.Text = destino{1};
    salidaFecha.Text = fecha{1};
end

`
            },
            {
                archivo: "P8",
                titulo: "Práctica 8",
                nombre: "Manejo de conocimiento incierto e incompleto",
                descripcion: `Pronóstico del Clima (Conocimiento Incierto)
Diseño un sistema de predicción del clima que estime la probabilidad de lluvia basándose
en datos como la humedad y la nubosidad, pero con un grado de incertidumbre.`,
                codigo: `
function interfaz_pronostico_clima
    % Crear la ventana principal
    fig = uifigure('Name', 'Pronóstico del Clima', 'Position', [100, 100, 250, 300]);

    uilabel(fig, 'Text', 'Pronóstico del Clima', 'Position', [70, 270, 250, 20], 'FontWeight','bold');

    % Desplegables para las variables
    uilabel(fig, 'Text', 'Humedad:', 'Position', [20, 220, 100, 20]);
    humedadMenu = uidropdown(fig, 'Items', {'alta', 'media', 'baja'}, 'Position', [120, 220, 100, 20]);

    uilabel(fig, 'Text', 'Nubosidad:', 'Position', [20, 180, 100, 20]);
    nubosidadMenu = uidropdown(fig, 'Items', {'densa', 'moderada', 'ligera'}, 'Position', [120, 180, 100, 20]);

    uilabel(fig, 'Text', 'Temperatura:', 'Position', [20, 140, 100, 20]);
    temperaturaMenu = uidropdown(fig, 'Items', {'alta', 'media', 'baja'}, 'Position', [120, 140, 100, 20]);

    % Botón para calcular
    calcularBtn = uibutton(fig, 'Text', 'Calcular', 'Position', [75, 80, 100, 40], ...
        'ButtonPushedFcn', @(btn, event) calcularProbabilidad(humedadMenu, nubosidadMenu, temperaturaMenu));

    % Botón de ayuda
    ayudaBtn = uibutton(fig, 'Text', '?', 'Position', [220, 10, 25, 25], ...
        'BackgroundColor',[0.4,0.63,1],...
        'FontColor',[0.87,0.87,0.87],...
        'FontWeight', 'bold',...
        'ButtonPushedFcn', @(btn, event) mostrarAyuda());

    % Etiqueta para mostrar resultados
    resultadoLabel = uilabel(fig, 'Text', '', 'Position', [0, 40, 250, 20], 'HorizontalAlignment', 'center');
    
    % Función de cálculo
    function calcularProbabilidad(humedadMenu, nubosidadMenu, temperaturaMenu)
        humedad = char(humedadMenu.Value);
        nubosidad = char(nubosidadMenu.Value);
        temperatura = char(temperaturaMenu.Value);

        prob_lluvia = pronostico_clima(humedad, nubosidad, temperatura);
        resultadoLabel.Text = sprintf('Probabilidad de lluvia: %.2f%%', prob_lluvia * 100);
    end

    % Función de ayuda
    function mostrarAyuda()
        % Crear ventana de ayuda
        ayudaFig = uifigure('Name', 'Ayuda', 'Position', [150, 150, 420, 300]);
        
        % Texto de ayuda
        ayudaText = [
            "Este programa calcula la probabilidad de lluvia basada en tres factores:"
            " - Humedad: Puede ser alta, media o baja."
            " - Nubosidad: Puede ser densa, moderada o ligera."
            " - Temperatura: Puede ser alta, media o baja."
            ""
            "Cálculo de la probabilidad:"
            "  El programa usa el Teorema de Bayes para combinar las probabilidades"
            "  individuales de cada factor, determinando la probabilidad conjunta"
            "  de que llueva dado el estado de las variables seleccionadas."
            ""
        ];
        
        uilabel(ayudaFig, 'Text', strjoin(ayudaText, '\n'), 'Position', [20, 20, 400, 260], ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

        % Cargar y mostrar la imagen
        ax = axes(ayudaFig, 'Position', [0.1 0.1 0.8 0.4]);  % Área donde se mostrará la imagen
        img = imread('img.png');  % Asegúrate de poner la ruta correcta a tu imagen
        imshow(img, 'Parent', ax);
    end
end

function prob_lluvia = pronostico_clima(humedad, nubosidad, temperatura)
    % Valores predeterminados si no se proporcionan argumentos
    if nargin < 3
        humedad = 'media';
        nubosidad = 'moderada';
        temperatura = 'media';
    end

    % Probabilidades iniciales
    P_lluvia = 0.3;
    P_no_lluvia = 0.7;

    % Probabilidades condicionales
    P_humedad_dada_lluvia = [0.8, 0.6, 0.2]; 
    P_humedad_dada_no_lluvia = [0.3, 0.4, 0.7];

    P_nubosidad_dada_lluvia = [0.9, 0.7, 0.4];
    P_nubosidad_dada_no_lluvia = [0.2, 0.5, 0.8];

    P_temperatura_dada_lluvia = [0.4, 0.5, 0.7];
    P_temperatura_dada_no_lluvia = [0.7, 0.5, 0.2];

    % Índices de las categorías
    humedad_idx = find(strcmp(humedad, {'alta', 'media', 'baja'}));
    nubosidad_idx = find(strcmp(nubosidad, {'densa', 'moderada', 'ligera'}));
    temperatura_idx = find(strcmp(temperatura, {'alta', 'media', 'baja'}));

    % Cálculo de probabilidades conjuntas
    P_dados_lluvia = P_humedad_dada_lluvia(humedad_idx) * ...
                     P_nubosidad_dada_lluvia(nubosidad_idx) * ...
                     P_temperatura_dada_lluvia(temperatura_idx);

    P_dados_no_lluvia = P_humedad_dada_no_lluvia(humedad_idx) * ...
                        P_nubosidad_dada_no_lluvia(nubosidad_idx) * ...
                        P_temperatura_dada_no_lluvia(temperatura_idx);

    % Teorema de Bayes
    P_lluvia_dados = P_dados_lluvia * P_lluvia / ...
                     (P_dados_lluvia * P_lluvia + P_dados_no_lluvia * P_no_lluvia);

    prob_lluvia = P_lluvia_dados;
end

`
            },
            {
                archivo: "P9",
                titulo: "Práctica 9",
                nombre: "Características de un conjunto de datos",
                descripcion: `Ejercicios propuestos
Desarrolle un programa para determinar el grado de eficiencia de un sensor detector de
presencia mediante infrarrojo. Use una interfaz gráfica con botón de ayuda.`,
                codigo: `
%% Programa para evaluar la eficiencia de un sensor infrarrojo

% Definir las variables
sensor = ["S1", "S2", "S3", "S4", "S5"];
entrada_prueba = [
    0,0,0,0,0; % Prueba 1
    1,1,1,1,1; % Prueba 2
    0,0,0,0,0; % Prueba 3
    1,1,1,1,1; % Prueba 4
    0,0,0,0,0; % Prueba 5
    1,1,1,1,1; % Prueba 6
    0,0,0,0,0; % Prueba 7
    1,1,1,1,1; % Prueba 8
    0,0,0,0,0; % Prueba 9
    1,1,1,1,1  % Prueba 10
];
salida_prueba = [
    0,0,0,0,1; % Prueba 1
    0,1,1,1,0; % Prueba 2
    0,0,0,0,0; % Prueba 3
    1,1,0,1,1; % Prueba 4
    0,0,0,0,0; % Prueba 5
    1,1,0,1,1; % Prueba 6
    1,0,0,1,1; % Prueba 7
    1,1,0,1,1; % Prueba 8
    0,0,0,0,1; % Prueba 9
    1,1,1,1,1  % Prueba 10
];

% Calcular detecciones correctas y eficiencia por sensor
detecciones_correctas = sum(entrada_prueba == salida_prueba, 1);
total_pruebas = size(entrada_prueba, 1);
eficiencia = (detecciones_correctas / total_pruebas) * 100;

% Crear la tabla de datos
datosSensores = table(sensor', detecciones_correctas', eficiencia', ...
    'VariableNames', {'Sensor', 'Detecciones_Correctas', 'Eficiencia'});

% Crear una aplicación básica con MATLAB App Designer
definirGUI();

function definirGUI()
    % Crear la figura principal
    fig = uifigure('Name', 'Eficiencia de Sensor Infrarrojo', 'Position', [100, 100, 600, 300]);

    % Crear tabla para mostrar los datos
    uitable(fig, 'Data', evalin('base', 'datosSensores'), ...
        'Position', [20, 120, 560, 145]);

    % Campo de texto para mostrar eficiencia promedio
    uilabel(fig, 'Position', [20, 20, 200, 22], 'Text', 'Eficiencia Promedio:');
    eficienciaLabel = uilabel(fig, 'Position', [150, 20, 100, 22], 'Text', '');

    % Botón para calcular eficiencia promedio
    calcularBtn = uibutton(fig, 'Text', 'Calcular Promedio', 'Position', [20, 50, 150, 22], ...
        'ButtonPushedFcn', @(btn, event) calcularPromedio(eficienciaLabel));

    % Botón de ayuda
    ayudaBtn = uibutton(fig, 'Text', '?', 'Position', [550, 20, 20, 20], ...
        'FontWeight', 'bold',...
        'ButtonPushedFcn', @(btn, event) mostrarAyuda());
end

function calcularPromedio(label)
    datos = evalin('base', 'datosSensores');
    promedio = mean(datos.Eficiencia);
    label.Text = sprintf('%.2f%%', promedio);
end

function mostrarAyuda()
    % Crear ventana de ayuda
    ayudaFig = uifigure('Name', 'Ayuda', 'Position', [150, 150, 650, 250]);

    % Texto detallado de ayuda
    msg = [
        "Este programa evalúa la eficiencia de sensores infrarrojos."
        ""
        "Funcionamiento:"
        "1. Los datos de entrada y salida se definen para simular pruebas realizadas a los sensores."
        "2. La eficiencia se calcula como el porcentaje de detecciones correctas realizadas por cada sensor."
        "3. Los resultados se presentan en una tabla que muestra las detecciones correctas y la eficiencia por sensor."
        ""
        "Estructura de las tablas:"
        "- La tabla contiene las columnas:"
        "    * Sensor: Identificador del sensor (S1, S2, ...)."
        "    * Detecciones_Correctas: Número de pruebas donde la salida coincidió con la entrada esperada."
        "    * Eficiencia: Porcentaje de aciertos calculado como (detecciones correctas / total de pruebas) * 100."
    ];

    uilabel(ayudaFig, 'Text', strjoin(msg, '\n'), 'Position', [20, 50, 650, 180], ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
end
`
            },
            {
                archivo: "P10",
                titulo: "Práctica 10",
                nombre: "Algoritmos clasificadores basados en distancia",
                descripcion: `Ejercicio propuesto
Desarrolle una aplicación usando un clasificador K-Vecinos Más Cercanos (K-NN) para
reconocer dígitos escritos a mano en el conjunto de datos MNIST. Este conjunto contiene
imágenes de dígitos (0-9), que es un caso típico para modelos de clasificación.
Para simplificar, usaremos una versión reducida de las imágenes de MNIST y las etiquetas
correspondientes en MATLAB.`,
                codigo: `
% Cargar el conjunto de datos MNIST reducido
load('mnist_reducido.mat'); 
% X: matriz de imágenes (n muestras x m características)
% Y: vector de etiquetas correspondientes (n x 1)

% Preprocesamiento de datos
X = double(X) / 255; % Normalizar los valores de píxeles a [0,1]

% Separar datos en entrenamiento (70%) y prueba (30%)
cv = cvpartition(Y, 'HoldOut', 0.3);
Xtrain = X(training(cv), :);
Ytrain = Y(training(cv), :);
Xtest = X(test(cv), :);
Ytest = Y(test(cv), :);

% Entrenar el clasificador K-Vecinos Más Cercanos (K-NN)
k = 5; % Número de vecinos
mdl = fitcknn(Xtrain, Ytrain, 'NumNeighbors', k, 'Distance', 'euclidean');

% Realizar predicciones en el conjunto de prueba
Ypred = predict(mdl, Xtest);

% Evaluar el rendimiento del clasificador
accuracy = sum(Ypred == Ytest) / length(Ytest) * 100;
fprintf('Precisión del clasificador K-NN: %.2f%%\n', accuracy);

% Mostrar la matriz de confusión
figure;
confusionchart(Ytest, Ypred);
title('Matriz de Confusión del Clasificador K-NN');

% Visualizar algunas predicciones
numExamples = 5;
figure;
for i = 1:numExamples
    subplot(1, numExamples, i);
    imshow(reshape(Xtest(i, :), [28, 28]), []); % Ajusta si el tamaño es diferente
    title(sprintf('Etiqueta: %d\nPredicción: %d', Ytest(i), Ypred(i)));
end

`
            },
            {
                archivo: "P11",
                titulo: "Práctica 11",
                nombre: "Árboles de decisión",
                descripcion: `Ejercicio propuesto
Programa una aplicación para clasificar pacientes como de "alto riesgo" o "bajo
riesgo" de desarrollar una enfermedad cardíaca en función de ciertas características,
como edad, presión arterial y colesterol. Implementa su interfaz gráfica y agrega su
botón de ayuda.`,
                codigo: `
% Generar datos sintéticos para pacientes
rng(1); % Para reproducibilidad
numSamples = 200; % Número total de pacientes

% Características:
% 1. Edad (años)
% 2. Presión arterial sistólica (mmHg)
% 3. Nivel de colesterol (mg/dL)

% Generar datos para pacientes de bajo riesgo
X_lowRisk = [randi([20, 40], numSamples/2, 1), ...  % Edad joven
             randi([90, 120], numSamples/2, 1), ... % Baja presión arterial
             randi([150, 200], numSamples/2, 1)];  % Nivel de colesterol normal

% Generar datos para pacientes de alto riesgo
X_highRisk = [randi([50, 80], numSamples/2, 1), ...  % Edad avanzada
              randi([130, 180], numSamples/2, 1), ... % Alta presión arterial
              randi([220, 300], numSamples/2, 1)];   % Alto colesterol

% Combinar datos
X = [X_lowRisk; X_highRisk];
Y = [repmat("Bajo Riesgo", numSamples/2, 1); repmat("Alto Riesgo", numSamples/2, 1)];

% Entrenar el árbol de decisión
tree = fitctree(X, Y);

% Crear la interfaz gráfica
f = figure('Name', 'Clasificación de Riesgo Cardíaco', 'Position', [100, 100, 600, 400]);

% Ejes para visualización de datos
ax = axes('Parent', f, 'Position', [0.1, 0.3, 0.8, 0.6]);
gscatter(X(:, 1), X(:, 2), Y, 'br', 'xo');
xlabel(ax, 'Edad');
ylabel(ax, 'Presión Arterial Sistólica');
title(ax, 'Datos Sintéticos de Pacientes');
legend(ax, 'Bajo Riesgo', 'Alto Riesgo');

% Botón de predicción
uicontrol('Style', 'text', 'Parent', f, 'Position', [50, 100, 100, 30], 'String', 'Edad:');
inputEdad = uicontrol('Style', 'edit', 'Parent', f, 'Position', [150, 100, 100, 30]);

uicontrol('Style', 'text', 'Parent', f, 'Position', [50, 60, 100, 30], 'String', 'Presión:');
inputPresion = uicontrol('Style', 'edit', 'Parent', f, 'Position', [150, 60, 100, 30]);

uicontrol('Style', 'text', 'Parent', f, 'Position', [50, 20, 100, 30], 'String', 'Colesterol:');
inputColesterol = uicontrol('Style', 'edit', 'Parent', f, 'Position', [150, 20, 100, 30]);

btnPredict = uicontrol('Style', 'pushbutton', 'Parent', f, 'Position', [300, 60, 100, 40], 'String', 'Predecir', ...
    'Callback', @(~, ~) predictRisk(inputEdad, inputPresion, inputColesterol, tree));

% Botón de ayuda
btnHelp = uicontrol('Style', 'pushbutton', 'Parent', f, 'Position', [450, 60, 100, 40], 'String', 'Ayuda', ...
    'Callback', @(~, ~) showHelp());

% Función para predecir el riesgo
function predictRisk(inputEdad, inputPresion, inputColesterol, tree)
    edad = str2double(get(inputEdad, 'String'));
    presion = str2double(get(inputPresion, 'String'));
    colesterol = str2double(get(inputColesterol, 'String'));
    if isnan(edad) || isnan(presion) || isnan(colesterol)
        msgbox('Por favor, ingrese valores válidos.', 'Error', 'error');
        return;
    end
    newPatient = [edad, presion, colesterol];
    predictedLabel = predict(tree, newPatient);
    msgbox(['El paciente es clasificado como: ', char(predictedLabel)], 'Resultado');
end

% Función para mostrar ayuda
function showHelp()
    helpText = ['Este programa utiliza un árbol de decisión para clasificar pacientes ', ...
                'en "Bajo Riesgo" o "Alto Riesgo" de desarrollar una enfermedad cardíaca.\n', ...
                'El algoritmo analiza características como la edad, presión arterial sistólica ', ...
                'y nivel de colesterol para realizar la clasificación.\n', ...
                'El árbol de decisión se entrena con datos sintéticos que simulan diferentes casos.'];
    msgbox(helpText, 'Ayuda');
end
`
            },
            {
                archivo: "P12",
                titulo: "Práctica 12",
                nombre: "Métodos de validación",
                descripcion: `Ejercicio propuesto
Desarrolle una aplicación para una Validación de un método de cromatografía líquida
de alta eficacia (HPLC) para determinar la pureza de un fármaco en una formulación
farmacéutica.`,
                codigo: `
function createHPLCApp()
    % Crear la figura principal de la aplicación
    fig = uifigure('Name', 'Validación de Método HPLC', 'Position', [100, 100, 800, 400]);

    % Panel de entrada de datos
    panelInputs = uipanel(fig, 'Title', 'Datos de Calibración', 'Position', [20, 150, 260, 200]);
    uilabel(panelInputs, 'Position', [10, 140, 240, 20], 'Text', 'Concentraciones del fármaco (mg/L):');
    inputFarmaco = uitextarea(panelInputs, 'Position', [10, 110, 240, 30], 'Value', {'10', '20', '30', '40', '50'});
    uilabel(panelInputs, 'Position', [10, 80, 240, 20], 'Text', 'Áreas del pico del fármaco:');
    inputAreaFarmaco = uitextarea(panelInputs, 'Position', [10, 50, 240, 30], 'Value', {'1.2', '2.4', '3.6', '4.8', '6.0'});

    % Área de resultados
    panelResultados = uipanel(fig, 'Title', 'Resultados', 'Position', [20, 20, 260, 120]);
    resultadoText = uitextarea(panelResultados, 'Position', [10, 10, 240, 90], ...
        'Editable', 'off', 'Value', {'Resultados aparecerán aquí.'});
    
    % Ejes para las gráficas
    axesFarmaco = uiaxes(fig, 'Position', [300, 100, 220, 260]);
    title(axesFarmaco, 'Curva de Calibración - Fármaco');
    xlabel(axesFarmaco, 'Concentración (mg/L)');
    ylabel(axesFarmaco, 'Área del Pico');
    
    axesImpureza = uiaxes(fig, 'Position', [550, 100, 220, 260]);
    title(axesImpureza, 'Curva de Calibración - Impureza');
    xlabel(axesImpureza, 'Concentración (mg/L)');
    ylabel(axesImpureza, 'Área del Pico');
    
    % Botón para calcular
    calcularBtn = uibutton(fig, 'Position', [300, 50, 100, 30], 'Text', 'Calcular', ...
        'ButtonPushedFcn', @(btn, event) calcularConcentraciones(inputFarmaco, inputAreaFarmaco));

    % Botón de ayuda
    ayudaBtn = uibutton(fig, 'Position', [450, 50, 100, 30], 'Text', 'Ayuda', ...
        'ButtonPushedFcn', @(btn, event) mostrarAyuda());

    % Función para calcular las concentraciones
function calcularConcentraciones(inputFarmaco, inputAreaFarmaco)
    try
        % Datos del fármaco
        concentracion_farmaco = str2double(split(inputFarmaco.Value, ','));
        area_pico_farmaco = str2double(split(inputAreaFarmaco.Value, ','));
        
        % Datos de calibración (pueden ser ajustados)
        concentracion_impureza = [1, 2, 3, 4, 5];
        area_pico_impureza = [0.1, 0.2, 0.3, 0.4, 0.5];
        
        % Calcular las curvas de calibración
        p_farmaco = polyfit(concentracion_farmaco, area_pico_farmaco, 1);
        p_impureza = polyfit(concentracion_impureza, area_pico_impureza, 1);
        
        % Ejemplo de valores medidos (pueden ser personalizados)
        areas_muestras_farmaco = [2, 3];
        areas_muestras_impureza = [0.15, 0.22];
        
        % Calcular concentraciones en las muestras
        conc_muestras_farmaco = (areas_muestras_farmaco - p_farmaco(2)) / p_farmaco(1);
        conc_muestras_impureza = (areas_muestras_impureza - p_impureza(2)) / p_impureza(1);
        
        % Mostrar los resultados
        resultadoText.Value = {
            'Concentraciones del fármaco en las muestras (mg/L):', ...
            sprintf('    %.4f    %.4f', conc_muestras_farmaco), ...
            '', ...
            'Concentraciones de las impurezas en las muestras (mg/L):', ...
            sprintf('    %.4f    %.4f', conc_muestras_impureza)
        };
        
        % Actualizar la gráfica del fármaco
        cla(axesFarmaco);
        plot(axesFarmaco, concentracion_farmaco, area_pico_farmaco, 'o', 'DisplayName', 'Datos');
        hold(axesFarmaco, 'on');
        f_linea_farmaco = polyval(p_farmaco, concentracion_farmaco);
        plot(axesFarmaco, concentracion_farmaco, f_linea_farmaco, 'r-', 'DisplayName', 'Ajuste Lineal');
        legend(axesFarmaco, 'show');
        hold(axesFarmaco, 'off');
        
        % Actualizar la gráfica de impurezas
        cla(axesImpureza);
        plot(axesImpureza, concentracion_impureza, area_pico_impureza, 'o', 'DisplayName', 'Datos');
        hold(axesImpureza, 'on');
        f_linea_impureza = polyval(p_impureza, concentracion_impureza);
        plot(axesImpureza, concentracion_impureza, f_linea_impureza, 'r-', 'DisplayName', 'Ajuste Lineal');
        legend(axesImpureza, 'show');
        hold(axesImpureza, 'off');
    catch
        resultadoText.Value = "Error en los datos ingresados. Verifique las entradas.";
    end
end

    % Función para mostrar la ayuda
    function mostrarAyuda()
        uialert(fig, ['Esta aplicación valida un método de cromatografía HPLC para determinar la pureza de un fármaco. ' ...
                      'Ingrese las concentraciones y áreas de los picos de calibración del fármaco en los campos correspondientes. ' ...
                      'Presione "Calcular" para obtener las curvas de calibración y concentraciones de las muestras.'], ...
                      'Ayuda');
    end
end
`
            },
            {
                archivo: "P13",
                titulo: "Práctica 13",
                nombre: "Algoritmos Genéticos",
                descripcion: `Ejercicio propuesto
Desarrolla una apliación para optimizar un portafolio de inversiones. En este caso, nuestro
objetivo es maximizar el retorno esperado de un portafolio, minimizando al mismo tiempo
el riesgo (medido por la varianza del portafolio).`,
                codigo: `
% Parámetros del algoritmo genético
numGeneraciones = 100;  % Número de generaciones
poblacionTamano = 50;   % Tamaño de la población
numActivos = 5;         % Número de activos
cruceProb = 0.8;        % Probabilidad de cruce
mutacionProb = 0.1;     % Probabilidad de mutación
lambda = 0.5;           % Peso para balancear retorno y riesgo

% Datos de los activos
retornos = [0.12, 0.18, 0.15, 0.10, 0.08]; % Retornos esperados
covarianza = [0.04, 0.02, 0.01, 0.01, 0.00;  % Matriz de covarianza
              0.02, 0.05, 0.02, 0.01, 0.01;
              0.01, 0.02, 0.03, 0.01, 0.00;
              0.01, 0.01, 0.01, 0.02, 0.01;
              0.00, 0.01, 0.00, 0.01, 0.01];

% Inicialización de la población
poblacion = rand(poblacionTamano, numActivos);
poblacion = poblacion ./ sum(poblacion, 2); % Normalizar para que las proporciones sumen 1

% Función de fitness
fitness_func = @(p) sum(p .* retornos) - lambda * (p * covarianza * p');

for generacion = 1:numGeneraciones
    % Evaluar fitness
    fitness = arrayfun(@(i) fitness_func(poblacion(i, :)), 1:poblacionTamano);

    % Selección: torneo
    nuevaPoblacion = zeros(size(poblacion));
    for i = 1:poblacionTamano
        ind1 = randi(poblacionTamano);
        ind2 = randi(poblacionTamano);
        if fitness(ind1) > fitness(ind2)
            nuevaPoblacion(i, :) = poblacion(ind1, :);
        else
            nuevaPoblacion(i, :) = poblacion(ind2, :);
        end
    end

    % Cruce
    for i = 1:2:poblacionTamano-1
        if rand < cruceProb
            puntoCruce = randi(numActivos-1);
            padre1 = nuevaPoblacion(i, :);
            padre2 = nuevaPoblacion(i+1, :);
            nuevaPoblacion(i, :) = [padre1(1:puntoCruce), padre2(puntoCruce+1:end)];
            nuevaPoblacion(i+1, :) = [padre2(1:puntoCruce), padre1(puntoCruce+1:end)];
        end
    end

    % Mutación
    for i = 1:poblacionTamano
        if rand < mutacionProb
            mutacionIdx = randi(numActivos);
            nuevaPoblacion(i, mutacionIdx) = rand;
            nuevaPoblacion(i, :) = nuevaPoblacion(i, :) / sum(nuevaPoblacion(i, :));
        end
    end

    % Actualizar población
    poblacion = nuevaPoblacion;

    % Mostrar el mejor fitness
    disp(['Generación ', num2str(generacion), ': Mejor fitness = ', num2str(max(fitness))]);
end

% Resultados finales
[~, mejorIdx] = max(fitness);
mejorSolucion = poblacion(mejorIdx, :);
disp('Mejor portafolio encontrado:');
disp(mejorSolucion);

`
            },
            {
                archivo: "P14",
                titulo: "Práctica 14",
                nombre: "Redes neuronales",
                descripcion: `Ejercicio propuesto
Desarrolle una aplicación para predecir precios de casas usando una red neuronal de
regresión. Vamos a trabajar con un conjunto de datos de precios de viviendas con
características como el tamaño de la casa, el número de habitaciones, el número de baños, y
la ubicación (esto último con valores numéricos o categorías para simplificar).
Este ejemplo se basa en una tarea de regresión, donde la red neuronal predice un valor
continuo (precio) en lugar de una clase.
Para este ejercicio, se va a generar un conjunto de datos sintético. Puede usar un
conjunto de datos real con características de viviendas.`,
                codigo: `
% Generar datos sintéticos
rng(1); % Reproducibilidad
numCasas = 500;
tamano = rand(numCasas, 1) * 200 + 50; % tamano en m² (50 a 250 m²)
habitaciones = randi([1, 5], numCasas, 1); % Número de habitaciones (1 a 5)
banos = randi([1, 3], numCasas, 1); % Número de banos (1 a 3)
ubicacion = randi([1, 3], numCasas, 1); % Zonas codificadas (1 a 3)

% Generar precios con ruido
precios = 50000 + 300 * tamano + 20000 * habitaciones + ...
          15000 * banos + 10000 * ubicacion + randn(numCasas, 1) * 10000;

% Crear matriz de características
X = [tamano, habitaciones, banos, ubicacion];
Y = precios;

% Dividir en conjunto de entrenamiento y prueba
cv = cvpartition(numCasas, 'HoldOut', 0.3); % 30% para prueba
idx = cv.test;

XTrain = X(~idx, :);
YTrain = Y(~idx, :);
XTest = X(idx, :);
YTest = Y(idx, :);

% Normalizar características
[XTrain, mu, sigma] = zscore(XTrain); % Media y desviación estándar
XTest = (XTest - mu) ./ sigma; % Normalizar conjunto de prueba

% Crear la red neuronal
net = fitrnet(XTrain, YTrain, ...
              'LayerSizes', [20, 10], ... % Dos capas ocultas con 20 y 10 neuronas
              'Activations', 'relu', ... % Función de activación ReLU
              'Standardize', true); % Estandarizar las características

% Predecir en el conjunto de prueba
YPred = predict(net, XTest);

% Evaluar el modelo
mse = mean((YPred - YTest).^2);
fprintf('Error cuadrático medio (MSE): %.2f\n', mse);

% Visualizar resultados
figure;
scatter(YTest, YPred, 'filled');
hold on;
plot([min(YTest), max(YTest)], [min(YTest), max(YTest)], 'r--');
title('Predicción de precios de casas');
xlabel('Precios reales');
ylabel('Precios predichos');
grid on;
legend('Predicciones', 'Línea ideal');
hold off;

`
            }
        ];

        const catalogo = document.getElementById('catalogo');

        // Generar tarjetas dinámicamente
        practicas.forEach((practica, index) => {
            const card = document.createElement('div');
            card.className = 'card';

            card.innerHTML = `
                <div class="card-body">
                    <h5 class="card-title">${practica.titulo}</h5>
                    <p class="card-text">${practica.nombre}</p>
                    <button class="btn btn-primary" onclick="abrirModal(${index})">Ver más</button>
                </div>
            `;
            catalogo.appendChild(card);
        });

        // Función para abrir el modal con detalles
        function abrirModal(index) {
            const practica = practicas[index];
            document.getElementById('modalPracticaLabel').textContent = `${practica.titulo}: ${practica.nombre}`;
            document.getElementById('descripcion').textContent = practica.descripcion;
            const codigoElement = document.getElementById('codigo').querySelector('code');
            codigoElement.textContent = practica.codigo;
            hljs.highlightElement(codigoElement);
            document.getElementById('descargar').href = `practicas/${practica.archivo}.zip`;
            new bootstrap.Modal(document.getElementById('modalPractica')).show();
        }
