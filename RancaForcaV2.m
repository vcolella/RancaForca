%%%%%%%%%%%%%%%%%% RANCA FORÇA V3 %%%%%%%%%%%%%%%%%%
%%% V3 : É MAIOR, É MELHOR, É A VERSÃO V2 ! (Revisada 86 vezes até o dado
%%% instante)
%%% O objetivo aqui é ler as strip forces que saem do AVL
%%% ESSE PROGRAMA TÁ NOJENTO mas operacional, carinho com ele.
%%% 
%%% Author : Victor C. G. C. Pereira
%%% Contact: victorcgcpereira@gmail.com
%%%
%%%%%%%%%%%%%%%%%
%% Inicialização

clc; close all; clearvars
GravarManobra = true;
% nomeDaCarta = '..\CARGAS.dat';
nomeDaCarta = '..\CARGASposProjetoULTIMACORRECAO1pontoZero.dat';
plotar = 1;
%% Parametros
FS = 1.0;   % Fator de segurança              
%% Manobras
% Sessao dedicada a gravar manobras que saíam do simulador_2017

% manobra(1).nome = 'ManobraCorrigidaProfundor4emeio';                                      	% Nome da manobra no .mat
% manobra(2).nome = 'ManobraCorrigidaLeme6';                                                	% Nome da manobra no .mat
% manobra(3).nome = 'ManobraCorrigidaAileron8';                                            	% Nome da manobra no .mat
% manobra(4).nome = 'PuxadaVc11';                                                             % Nome da manobra no .mat
% manobra(5).nome = 'PullDownVc8';                                                        	% Nome da manobra no .mat
% manobra(6).nome = 'PullUpVc8';                                                              % Nome da manobra no .mat
% manobra(7).nome = 'CurvaVento4emV_8leme_6aileron_Pulso';                                  	% Nome da manobra no .mat
% manobra(8).nome = 'PuxadaVdNova(25,88)';                                                    % Nome da manobra no .mat
% % manobra(1).nome = 'AileronSenoidal';


%% Begin
for iii = 1:numel(manobra)
    clearvars -except GravarManobra plotar FS coordSyst iii manobra nomeDaCarta
    load(['..\Manobras\' manobra(iii).nome '.mat']);                       	% Carrega parametros da manobra
    coordSyst = 0;                                                         	% Sistema de coordenadas de referência do modelo 
    firstWingNode = 35000;                                               	% Primeiro nó de carga da asa
    firstTailNode = 36000;                                                 	% Primeiro nó de carga da empenagem horizontal
    
    % Matriz de rotação para um ângulo de diedro theta
    Transf = @(theta) [1,   0           ,   0;
                       0,   cos(theta)  ,   sin(theta);
                       0,   -sin(theta) ,   cos(theta)];     
    
    thicknessPerfil = 12.4e-2; % Para posicionar o nó de carga na meia altura do perfil
    
    fileNameAviao = '..\..\..\Estabilidade e Controle\Estabilidade 2017\planeAVL.avl';	% Arquivo .avl do aviao
    
    
    %% Análise da Manobra
    [Lwd,Lwe,Mwd,Mwe,Ltd,Lte,Mtd,Mte] = deal(NaN(numel(Casa(1).aread),numel(time))); % Alocacao
    % [tcritLwd,tcritLwe,tcritMwd,tcritMwe,tcritLtd,tcritLte,tcritMtd,tcritMte] = deal(NaN(numel(Casa(1).aread),1)); % Alocacao
    for i=1:numel(qdin)
        L = [Ctotal(:).CL].*qdin.*S;
        Lwd(:,i) = Casa(i).cl_normd.*Casa(i).aread.*qdin(i);
        Lwe(:,i) = Casa(i).cl_norme.*Casa(i).areae.*qdin(i);
        
        Mwd(:,i) = Casa(i).cm_cd.*Casa(i).aread.*qdin(i);
        Mwe(:,i) = Casa(i).cm_ce.*Casa(i).areae.*qdin(i);
        
        Ltd(:,i) = Ce(i).cl_normd.*Ce(i).aread.*qdin(i);
        Lte(:,i) = Ce(i).cl_norme.*Ce(i).areae.*qdin(i);
        
        Mtd(:,i) = Ce(i).cm_cd.*Ce(i).aread.*qdin(i);
        Mte(:,i) = Ce(i).cm_ce.*Ce(i).areae.*qdin(i);
    end
    
    % Encontra instantes de máximos em cada painel
    tempo = [];
    for i=1:numel(Casa(1).aread)
        [~,tcritLwd] = find(abs(Lwd) == max(abs(Lwd(i,:))));
        [~,tcritLwe] = find(abs(Lwe) == max(abs(Lwe(i,:))));
        
        [~,tcritMwd] = find(abs(Mwd) == max(abs(Mwd(i,:))));
        [~,tcritMwe] = find(abs(Mwe) == max(abs(Mwe(i,:))));
        
        [~,tcritLtd] = find(abs(Ltd) == max(abs(Ltd(i,:))));
        [~,tcritLte] = find(abs(Lte) == max(abs(Lte(i,:))));
        
        [~,tcritMtd] = find(abs(Mtd) == max(abs(Mwd(i,:))));
        [~,tcritMte] = find(abs(Mte) == max(abs(Mwe(i,:))));
        
        tempo =[tempo;...
            tcritLwd;
            tcritLwe;
            tcritMwd;
            tcritMwe;
            tcritLtd;
            tcritLte;
            tcritMtd;
            tcritMte;];
        
    end
    tcrit = unique(tempo); % Indices dos instantes críticos da manobra
    %% Construindo caso a caso
    
    for j=1:numel(tcrit)
        %%%%%%%%%%%%%%%%%%%%%%% Preparação da ASA %%%%%%%%%%%%%%%%%%%%%%%%
        NoBaseAsa = firstWingNode;
        jAsa = [Casa(tcrit(j)).je;Casa(tcrit(j)).jd];
        YleAsa = [Casa(tcrit(j)).Ylee;Casa(tcrit(j)).Yled];
        ChordAsa = [Casa(tcrit(j)).chorde;Casa(tcrit(j)).chordd];
        AreaAsa = [Casa(tcrit(j)).areae;Casa(tcrit(j)).aread];
        clAsa = [Casa(tcrit(j)).cl_norme;Casa(tcrit(j)).cl_normd];
        cdAsa = [Casa(tcrit(j)).cd_norme;Casa(tcrit(j)).cd_normd];
        cm_c4Asa = [Casa(tcrit(j)).cm_ce;Casa(tcrit(j)).cm_cd];
        
        %%%%%%%%%%%%%%%%%%%%%%% Preparação da V-TAIL %%%%%%%%%%%%%%%%%%%%%%%%
        NoBaseVtail = firstTailNode;
        jVtail = [Ce(tcrit(j)).je;Ce(tcrit(j)).jd];
        YleVtail = [Ce(tcrit(j)).Ylee;Ce(tcrit(j)).Yled];
        ChordVtail = [Ce(tcrit(j)).chorde;Ce(tcrit(j)).chordd];
        AreaVtail = [Ce(tcrit(j)).areae;Ce(tcrit(j)).aread];
        clVtail = [Ce(tcrit(j)).cl_norme;Ce(tcrit(j)).cl_normd];
        cdVtail = [Ce(tcrit(j)).cd_norme;Ce(tcrit(j)).cd_normd];
        cm_c4Vtail = [Ce(tcrit(j)).cm_ce;Ce(tcrit(j)).cm_cd];
        
        Q = qdin(tcrit(j));
        %     figure
        %     teste = [flipud(YleAsa(21:end));YleAsa(1:20)];
        %     testeB = [flipud(clAsa(21:end));clAsa(1:20)];
        %     testeC = [flipud(AreaAsa(21:end));AreaAsa(1:20)];
        %     plot(teste+teste(end),testeB.*testeC.*Q,'LineWidth',1.6);
        %     xlabel('Envergadura [m]','FontWeight','bold','FontSize',16);
        %     ylabel('Sustentação [N]','FontWeight','bold','FontSize',16);
        %% Alocações
        [GridAsa,GridVtail] = deal(struct([]));
        %%  GridNodes da ASA
        %%%%%%%%%%%%%%%%%%%%%%% Lendo o arquivo .avl para coletar transições %%%%%%%%%%%%%%%%%%%%%%%%
        
        fileAviao = fopen(fileNameAviao,'r');
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',37);
        [XAsa(1),YAsa(1),ZAsa(1)] = deal(data{1},data{2},data{3});
        
        frewind(fileAviao);
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',45);
        [XAsa(2),YAsa(2),ZAsa(2)] = deal(data{1},data{2},data{3});
        
        frewind(fileAviao);
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',57);
        [XAsa(3),YAsa(3),ZAsa(3)] = deal(data{1},data{2},data{3});
        
        frewind(fileAviao);
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',69);
        [XAsa(4),YAsa(4),ZAsa(4)] = deal(data{1},data{2},data{3});
        
        frewind(fileAviao);
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',29);
        [XtranslateAsa,YtranslateAsa,ZtranslateAsa] = deal(data{1},data{2},data{3});
        
        % Polinômios para interpolar x na envergadura 
        px1Asa = polyfit(YAsa(1:2),XAsa(1:2),1);
        px2Asa = polyfit(YAsa(2:3),XAsa(2:3),1);
        px3Asa = polyfit(YAsa(3:end),XAsa(3:end),1);
        
        % Polinômios para interpolar z na envergadura 
        pz1Asa = polyfit(YAsa(1:2),ZAsa(1:2),1);
        pz2Asa = polyfit(YAsa(2:3),ZAsa(2:3),1);
        pz3Asa = polyfit(YAsa(3:end),ZAsa(3:end),1);
        
        %%%%%%%%%%%%%%%%%%%%%%% Calcula nós, forças e momentos %%%%%%%%%%%%%%%%%%%%%%%%
        for i=1:numel(jAsa)
            GridAsa(i).number = NoBaseAsa;
            NoBaseAsa = NoBaseAsa + 1;
            if abs(YleAsa(i))<YAsa(2)
                GridAsa(i).x = polyval(px1Asa,abs(YleAsa(i))) + 0.25*ChordAsa(i);
                GridAsa(i).y = polyval(pz1Asa,abs(YleAsa(i))) + .5*thicknessPerfil*ChordAsa(i);
                GridAsa(i).theta = atan(abs((ZAsa(2)-ZAsa(1))/(YAsa(2)-YAsa(1))))*(-sign(YleAsa(i)));
            end
            if abs(YleAsa(i))>YAsa(2) && abs(YleAsa(i))<YAsa(3)
                GridAsa(i).x = polyval(px2Asa,abs(YleAsa(i))) + 0.25*ChordAsa(i);
                GridAsa(i).y = polyval(pz2Asa,abs(YleAsa(i))) + .5*thicknessPerfil*ChordAsa(i);
                GridAsa(i).theta = atan(abs((ZAsa(3)-ZAsa(2))/(YAsa(3)-YAsa(2))))*(-sign(YleAsa(i)));
            end
            if abs(YleAsa(i))>YAsa(3)
                GridAsa(i).x = polyval(px3Asa,abs(YleAsa(i))) + 0.25*ChordAsa(i);
                GridAsa(i).y = polyval(pz3Asa,abs(YleAsa(i))) + .5*thicknessPerfil*ChordAsa(i);
                GridAsa(i).theta = atan(abs((ZAsa(end)-ZAsa(3))/(YAsa(end)-YAsa(3))))*(-sign(YleAsa(i)));
            end
            GridAsa(i).z =  YleAsa(i);
            
            GridAsa(i).force = Transf(GridAsa(i).theta)*[cdAsa(i)*AreaAsa(i)*Q;clAsa(i)*AreaAsa(i)*Q;0];
            GridAsa(i).moment = Transf(GridAsa(i).theta)*[0;0;-cm_c4Asa(i)*AreaAsa(i)*Q*ChordAsa(i)];
        end
        %%  GridNodes da VTAIL
        frewind(fileAviao);
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',104);        % <<< MUDEI HEARDELINES POIS ATUALIZEI O planeAVL.avl
        [XVtail(1),YVtail(1),ZVtail(1)] = deal(data{1},data{2},data{3});
        frewind(fileAviao);
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',120);        % <<< MUDEI HEARDELINES POIS ATUALIZEI O planeAVL.avl
        [XVtail(2),YVtail(2),ZVtail(2)] = deal(data{1},data{2},data{3});
        frewind(fileAviao);
        data = textscan(fileAviao,'%f %f %f %f %f\n','Headerlines',88);         % <<< MUDEI HEARDELINES POIS ATUALIZEI O planeAVL.avl
        [XtranslateVtail,YtranslateVtail,ZtranslateVtail] = deal(data{1},data{2},data{3});
        
        
        
        px1Vtail = polyfit(YVtail(1:2),XVtail(1:2),1);
        pz1Vtail = polyfit(YVtail(1:2),ZVtail(1:2),1);
        
        for i=1:numel(jVtail)
            GridVtail(i).number = NoBaseVtail;
            NoBaseVtail = NoBaseVtail + 1;
            
            GridVtail(i).x = polyval(px1Vtail,abs(YleVtail(i))) + 0.25*ChordVtail(i) + XtranslateVtail - XtranslateAsa;
            GridVtail(i).y = polyval(pz1Vtail,abs(YleVtail(i))) + .5*thicknessPerfil*ChordVtail(i);
            GridVtail(i).theta = atan(abs((ZVtail(end)-ZVtail(1))/(YVtail(end)-YVtail(1))))*(-sign(YleVtail(i)));
            
            GridVtail(i).z =  YleVtail(i);
            
            GridVtail(i).force = Transf(GridVtail(i).theta)*[cdVtail(i)*AreaVtail(i)*Q;clVtail(i)*AreaVtail(i)*Q;0];
            GridVtail(i).moment = Transf(GridVtail(i).theta)*[0;0;-cm_c4Vtail(i)*AreaVtail(i)*Q*ChordVtail(i)];
        end
        fclose all;
        %% Cartão NASTRAN
        if GravarManobra
            caseName = [manobra(iii).nome '_' num2str(time(tcrit(j))) 's_' num2str(L(tcrit(j))/(TOW*g)) ];
            load contadores
            loadSet = loadSet + 1;
            save('contadores.mat','loadSet');
            fileName = nomeDaCarta;
            fileNastran = fopen(fileName,'a');
            fprintf(fileNastran,['$ Femap with NX Nastran Load Set %d : ' caseName '\n'],loadSet);
            
            %%% ASA
            fprintf(fileNastran,'$\n$\n$ GRID POINTS DAS CARGAS NA ASA\n$\n');
            for i=1:numel(jAsa);
                fprintf(fileNastran,'%-8s%8.0d%8.0d%8.4f%8.4f%8.4f%8.4f\n','GRID',GridAsa(i).number,coordSyst,GridAsa(i).x,GridAsa(i).y,GridAsa(i).z,0);
            end
            fprintf(fileNastran,'$\n$\n$ FORCAS NA ASA\n$\n');
            for i=1:numel(jAsa);
                fprintf(fileNastran,'%-8s%8.0d%8.0d%8.0d%8.4f%8.4f%8.4f%8.4f\n','FORCE',loadSet,GridAsa(i).number,coordSyst,FS,GridAsa(i).force(1),GridAsa(i).force(2),GridAsa(i).force(3));
            end
            fprintf(fileNastran,'$\n$\n$ MOMENTOS NA ASA\n$\n');
            for i=1:numel(jAsa);
                fprintf(fileNastran,'%-8s%8.0d%8.0d%8.0d%8.4f%8.4f%8.4f%8.4f\n','MOMENT',loadSet,GridAsa(i).number,coordSyst,FS,GridAsa(i).moment(1),GridAsa(i).moment(2),GridAsa(i).moment(3));
            end
            %%% VTAIL
            fprintf(fileNastran,'$\n$\n$ GRID POINTS DAS CARGAS NA VTAIL\n$\n');
            for i=1:numel(jVtail);
                fprintf(fileNastran,'%-8s%8.0d%8.0d%8.4f%8.4f%8.4f%8.4f\n','GRID',GridVtail(i).number,coordSyst,GridVtail(i).x,GridVtail(i).y,GridVtail(i).z,0);
            end
            fprintf(fileNastran,'$\n$\n$ FORCAS NA VTAIL\n$\n');
            for i=1:numel(jVtail);
                fprintf(fileNastran,'%-8s%8.0d%8.0d%8.0d%8.4f%8.4f%8.4f%8.4f\n','FORCE',loadSet,GridVtail(i).number,coordSyst,FS,GridVtail(i).force(1),GridVtail(i).force(2),GridVtail(i).force(3));
            end
            fprintf(fileNastran,'$\n$\n$ MOMENTOS NA VTAIL\n$\n');
            for i=1:numel(jVtail);
                fprintf(fileNastran,'%-8s%8.0d%8.0d%8.0d%8.4f%8.4f%8.4f%8.4f\n','MOMENT',loadSet,GridVtail(i).number,coordSyst,FS,GridVtail(i).moment(1),GridVtail(i).moment(2),GridVtail(i).moment(3));
            end
            fclose all;
            fprintf('\nInstante %.3fs , %.2f g''s DONE',time(tcrit(j)),L(tcrit(j))/(TOW*g));
            %     teste = cell2mat(arrayfun(@(c) c.force(2), GridAsa(:), 'Uniform',0));
            fprintf('\nL = %f N\n',L(tcrit(j)));
            % teste2 = cell2mat(arrayfun(@(c) c.force(2), GridVtail(:), 'Uniform',0));
        end
    end
end
fprintf('\n');

%% Plot Asa
if plotar
    figure('Name','Asa')  % Pra escolher qual instante eu quero
    for j=1:numel(tcrit)
        subplot(round(sqrt(numel(tcrit)))+1,round(sqrt(numel(tcrit))),j)
        
        jAsa = [Casa(tcrit(j)).je;Casa(tcrit(j)).jd];
        YleAsa = [Casa(tcrit(j)).Ylee;Casa(tcrit(j)).Yled];
        ChordAsa = [Casa(tcrit(j)).chorde;Casa(tcrit(j)).chordd];
        AreaAsa = [Casa(tcrit(j)).areae;Casa(tcrit(j)).aread];
        clAsa = [Casa(tcrit(j)).cl_norme;Casa(tcrit(j)).cl_normd];
        cdAsa = [Casa(tcrit(j)).cd_norme;Casa(tcrit(j)).cd_normd];
        cm_c4Asa = [Casa(tcrit(j)).cm_ce;Casa(tcrit(j)).cm_cd];
        
        Q = qdin(tcrit(j));
        teste = [flipud(YleAsa(21:end));YleAsa(1:20)];
        testeB = [flipud(clAsa(21:end));clAsa(1:20)];
        testeC = [flipud(AreaAsa(21:end));AreaAsa(1:20)];
        plot(teste+teste(end),testeB.*testeC.*Q,'LineWidth',1.6);
        titulo = [num2str(time(tcrit(j))) 's'];
        xlabel(titulo,'FontWeight','bold','FontSize',10);
        grid minor
        %     ylabel('Sustentação [N]','FontWeight','bold','FontSize',16);
    end
    
    
    
    
    j = 2;
    titulo = ['Asa no instante ' num2str(time(tcrit(j))) ' s'];
    figure('Name',titulo);
    jAsa = [Casa(tcrit(j)).je;Casa(tcrit(j)).jd];
    YleAsa = [Casa(tcrit(j)).Ylee;Casa(tcrit(j)).Yled];
    ChordAsa = [Casa(tcrit(j)).chorde;Casa(tcrit(j)).chordd];
    AreaAsa = [Casa(tcrit(j)).areae;Casa(tcrit(j)).aread];
    clAsa = [Casa(tcrit(j)).cl_norme;Casa(tcrit(j)).cl_normd];
    cdAsa = [Casa(tcrit(j)).cd_norme;Casa(tcrit(j)).cd_normd];
    cm_c4Asa = [Casa(tcrit(j)).cm_ce;Casa(tcrit(j)).cm_cd];
    
    Q = qdin(tcrit(j));
    teste = [flipud(YleAsa(21:end));YleAsa(1:20)];
    testeB = [flipud(clAsa(21:end));clAsa(1:20)];
    testeC = [flipud(AreaAsa(21:end));AreaAsa(1:20)];
    plot(teste+teste(end),testeB.*testeC.*Q,'LineWidth',1.6);
    xlabel('Envergadura [m]','FontWeight','bold','FontSize',16);
    ylabel('Sustentação [N]','FontWeight','bold','FontSize',16);
    grid minor
    
    
    jj = [2 11];
    figure
    for ii = 1:numel(jj)
        j = jj(ii);jAsa = [Casa(tcrit(j)).je;Casa(tcrit(j)).jd];
        YleAsa = [Casa(tcrit(j)).Ylee;Casa(tcrit(j)).Yled];
        ChordAsa = [Casa(tcrit(j)).chorde;Casa(tcrit(j)).chordd];
        AreaAsa = [Casa(tcrit(j)).areae;Casa(tcrit(j)).aread];
        clAsa = [Casa(tcrit(j)).cl_norme;Casa(tcrit(j)).cl_normd];
        cdAsa = [Casa(tcrit(j)).cd_norme;Casa(tcrit(j)).cd_normd];
        cm_c4Asa = [Casa(tcrit(j)).cm_ce;Casa(tcrit(j)).cm_cd];
        
        Q = qdin(tcrit(j));
        teste = [flipud(YleAsa(21:end));YleAsa(1:20)];
        testeB = [flipud(clAsa(21:end));clAsa(1:20)];
        testeC = [flipud(AreaAsa(21:end));AreaAsa(1:20)];
        testeD = [flipud(ChordAsa(21:end));ChordAsa(1:20)];
        testeE = [flipud(cm_c4Asa(21:end));cm_c4Asa(1:20)];
        
        subplot(1,3,ii);
        
        plot(teste+teste(end),testeB.*testeC.*Q,'LineWidth',1.6);
        titulo = [num2str(time(tcrit(j))) 's'];
        title(titulo);
        xlabel('Envergadura [m]','FontWeight','bold','FontSize',16);
        ylabel('Sustentação [N]','FontWeight','bold','FontSize',16);
        grid minor
    end
    
    figure
    for ii = 1:numel(jj)
        j = jj(ii);
        jAsa = [Casa(tcrit(j)).je;Casa(tcrit(j)).jd];
        YleAsa = [Casa(tcrit(j)).Ylee;Casa(tcrit(j)).Yled];
        ChordAsa = [Casa(tcrit(j)).chorde;Casa(tcrit(j)).chordd];
        AreaAsa = [Casa(tcrit(j)).areae;Casa(tcrit(j)).aread];
        clAsa = [Casa(tcrit(j)).cl_norme;Casa(tcrit(j)).cl_normd];
        cdAsa = [Casa(tcrit(j)).cd_norme;Casa(tcrit(j)).cd_normd];
        cm_c4Asa = [Casa(tcrit(j)).cm_ce;Casa(tcrit(j)).cm_cd];
        
        Q = qdin(tcrit(j));
        teste = [flipud(YleAsa(21:end));YleAsa(1:20)];
        testeB = [flipud(clAsa(21:end));clAsa(1:20)];
        testeC = [flipud(AreaAsa(21:end));AreaAsa(1:20)];
        testeD = [flipud(ChordAsa(21:end));ChordAsa(1:20)];
        testeE = [flipud(cm_c4Asa(21:end));cm_c4Asa(1:20)];
        
        subplot(1,3,ii);
        plot(teste+teste(end),testeE.*testeC.*testeD.*Q,'LineWidth',1.6);
        titulo = [num2str(time(tcrit(j))) 's'];
        title(titulo);
        xlabel('Envergadura [m]','FontWeight','bold','FontSize',16);
        ylabel('Momento [N.m]','FontWeight','bold','FontSize',16);
        grid minor
    end
    
    
    
    
    %% Plot Cauda
    
    
    figure('Name','Vtail')  % Pra escolher qual instante eu quero
    for j=1:numel(tcrit)
        subplot(round(sqrt(numel(tcrit)))+1,round(sqrt(numel(tcrit))),j)
        %%%%%%%%%%%%%%%%%%%%%%% Preparação da V-TAIL %%%%%%%%%%%%%%%%%%%%%%%%
        jVtail = [Ce(tcrit(j)).je;Ce(tcrit(j)).jd];
        YleVtail = [Ce(tcrit(j)).Ylee;Ce(tcrit(j)).Yled];
        ChordVtail = [Ce(tcrit(j)).chorde;Ce(tcrit(j)).chordd];
        AreaVtail = [Ce(tcrit(j)).areae;Ce(tcrit(j)).aread];
        clVtail = [Ce(tcrit(j)).cl_norme;Ce(tcrit(j)).cl_normd];
        cdVtail = [Ce(tcrit(j)).cd_norme;Ce(tcrit(j)).cd_normd];
        cm_c4Vtail = [Ce(tcrit(j)).cm_ce;Ce(tcrit(j)).cm_cd];
        
        Q = qdin(tcrit(j));
        teste = [flipud(YleVtail(21:end));YleVtail(1:20)];
        testeB = [flipud(clVtail(21:end));clVtail(1:20)];
        testeC = [flipud(AreaVtail(21:end));AreaVtail(1:20)];
        plot(teste+teste(end),testeB.*testeC.*Q,'LineWidth',1.6);
        titulo = [num2str(time(tcrit(j))) 's'];
        xlabel(titulo,'FontWeight','bold','FontSize',10);
        grid minor
    end
    
    
    j = 2;
    titulo = ['Vtail no instante ' num2str(time(tcrit(j))) ' s'];
    figure('Name',titulo);
    jVtail = [Ce(tcrit(j)).je;Ce(tcrit(j)).jd];
    YleVtail = [Ce(tcrit(j)).Ylee;Ce(tcrit(j)).Yled];
    ChordVtail = [Ce(tcrit(j)).chorde;Ce(tcrit(j)).chordd];
    AreaVtail = [Ce(tcrit(j)).areae;Ce(tcrit(j)).aread];
    clVtail = [Ce(tcrit(j)).cl_norme;Ce(tcrit(j)).cl_normd];
    cdVtail = [Ce(tcrit(j)).cd_norme;Ce(tcrit(j)).cd_normd];
    cm_c4Vtail = [Ce(tcrit(j)).cm_ce;Ce(tcrit(j)).cm_cd];
    
    Q = qdin(tcrit(j));
    teste = [flipud(YleVtail(21:end));YleVtail(1:20)];
    testeB = [flipud(clVtail(21:end));clVtail(1:20)];
    testeC = [flipud(AreaVtail(21:end));AreaVtail(1:20)];
    plot(teste+teste(end),testeB.*testeC.*Q,'LineWidth',1.6);
    xlabel('Envergadura [m]','FontWeight','bold','FontSize',16);
    ylabel('Sustentação [N]','FontWeight','bold','FontSize',16);
    grid minor
end
