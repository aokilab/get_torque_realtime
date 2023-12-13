syms A B C D E F G;

T1 = [cos(A) -sin(A) 0 0 ;sin(A) cos(A) 0 0 ;0 0 1 130 ;0 0 0 1];
T2 = [cos(B) -sin(B) 0 0 ;0 0 1 0 ;-sin(B) -cos(B) 0 115 ;0 0 0 1];
T3 = [cos(C) -sin(C) 0 0 ;0 0 -1 -190 ;sin(C) cos(C) 0 0 ;0 0 0 1];
T4 = [cos(D) -sin(D) 0 0 ;0 0 1 0 ;-sin(D) -cos(D) 0 110 ;0 0 0 1];
T5 = [cos(E) -sin(E) 0 0 ;0 0 -1 -210 ;sin(E) cos(E) 0 0 ;0 0 0 1];
T6 = [cos(F) -sin(F) 0 0 ;0 0 1 0 ;-sin(F) -cos(F) 0 90 ;0 0 0 1];
T7 = [cos(G) -sin(G) 0 0 ;0 0 -1 -170 ;sin(G) cos(G) 0 0 ;0 0 0 1];
T8 = [1 0 0 0 ;0 1 0 0 ;0 0 1 888 ;0 0 0 1];

Ta = T1 * T2 * T3 * T4 * T5 * T6 * T7 *T8;

% 行列をセル配列に変換
cell = num2cell(Ta);

% 各要素を変数に代入
[R11 ,R12 ,R13 ,R14 ,R21 ,R22 ,R23 ,R24 ,R31 ,R32 ,R33 ,R34 ,R41 ,R42 ,R43 ,R44] = deal(cell{:});

%Roll,Pitch,Yawを計算
R = atan2(R23,R13);
P = atan2(sqrt(R13.^2+R23.^2),R33);
Y = atan2(R32,-R31);

%ヤコビ行列Jrを計算
Jr11 = diff(R14,A);
Jr12 = diff(R14,B);
Jr13 = diff(R14,C);
Jr14 = diff(R14,D);
Jr15 = diff(R14,E);
Jr16 = diff(R14,F);
Jr17 = diff(R14,G);

Jr21 = diff(R24,A);
Jr22 = diff(R24,B);
Jr23 = diff(R24,C);
Jr24 = diff(R24,D);
Jr25 = diff(R24,E);
Jr26 = diff(R24,F);
Jr27 = diff(R24,G);

Jr31 = diff(R34,A);
Jr32 = diff(R34,B);
Jr33 = diff(R34,C);
Jr34 = diff(R34,D);
Jr35 = diff(R34,E);
Jr36 = diff(R34,F);
Jr37 = diff(R34,G);

Jr41 = diff(R,A);
Jr42 = diff(R,B);
Jr43 = diff(R,C);
Jr44 = diff(R,D);
Jr45 = diff(R,E);
Jr46 = diff(R,F);
Jr47 = diff(R,G);

Jr51 = diff(P,A);
Jr52 = diff(P,B);
Jr53 = diff(P,C);
Jr54 = diff(P,D);
Jr55 = diff(P,E);
Jr56 = diff(P,F);
Jr57 = diff(P,G);

Jr61 = diff(Y,A);
Jr62 = diff(Y,B);
Jr63 = diff(Y,C);
Jr64 = diff(Y,D);
Jr65 = diff(Y,E);
Jr66 = diff(Y,F);
Jr67 = diff(Y,G);

Jr = [Jr11 Jr12 Jr13 Jr14 Jr15 Jr16 Jr17 ;
      Jr21 Jr22 Jr23 Jr24 Jr25 Jr26 Jr27 ;
      Jr31 Jr32 Jr33 Jr34 Jr35 Jr36 Jr37 ;
      Jr41 Jr42 Jr43 Jr44 Jr45 Jr46 Jr47 ;
      Jr51 Jr52 Jr53 Jr54 Jr55 Jr56 Jr57 ;
      Jr61 Jr62 Jr63 Jr64 Jr65 Jr66 Jr67 ];

%ヤコビ行列Jvを計算
H = [1 0 0 0 0       0 ;
     0 1 0 0 0       0 ;
     0 0 1 0 0       0 ;
     0 0 0 0 -sin(R) cos(R)*sin(P) ;
     0 0 0 0 cos(R)  sin(R)*sin(P) ;
     0 0 0 1 0       cos(P)];

Jv = mtimes(H,Jr);

save('jacobian.mat',"Jv")



