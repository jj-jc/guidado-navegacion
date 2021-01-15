function [vf,vg]=control_reactivo(ultrasonidoI,ultrasonidoF,ultrasonidoD)
%Se toman las medidas de cada uno de los ultrasonidos
    izquierda=apoloGetUltrasonicSensor(ultrasonidoI);
    frente=apoloGetUltrasonicSensor(ultrasonidoF);
    derecha=apoloGetUltrasonicSensor(ultrasonidoD);
    %Se inicializan las velocidades en cada iteración.
     vf=0;vg=0;
    %Condición de precuación
    if (frente < 0.40)||(derecha<0.40)||(izquierda<0.40)     
        if(frente < 0.15)||(derecha<0.15)||(izquierda<0.15) %Condición de peligro de choque
            vf=0;
            if izquierda < 0.1
                vg=-0.5/izquierda;
            else
                vg=0.5/derecha;
            end          
        elseif  izquierda < 0.40
            vg=-0.5/izquierda;
            vf=0;
        elseif derecha < 0.40
            vg=0.5/derecha;
            vf=0;
        else
            vf=0.1;
        end
    else       %Condición natural
        vf=10;
        vg=10;
    end
end