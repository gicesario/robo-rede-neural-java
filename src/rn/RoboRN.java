package rn;

import java.io.File;

import org.neuroph.core.NeuralNetwork;

import coppelia.FloatWA;
import coppelia.remoteApi;

public class RoboRN {

    public static boolean nao_achou_alvo = false;

    public static void main(String[] args) throws Exception {
        System.out.println("Início...");
        remoteApi vrep = new remoteApi();
        
        vrep.simxFinish(-1); 
        
        int conexaoVREPId = vrep.simxStart("127.0.0.1", 19999, true, true, 5000, 5); // socket v-rep
        
        if (conexaoVREPId == -1) {
        	System.err.println("Comunicacao com V-REP não encontrada!");
        }
        else {
            System.out.println("Conexao com V-REP efetuada.");   
            // enable the synchronous mode on the client:
            vrep.simxSynchronous(conexaoVREPId,true);
            
            // Carrega diretorio com o mesmo caminho da main            
            String dirRN = new File(RoboRN.class.getProtectionDomain().getCodeSource().getLocation().toURI()).getPath();
            String dirArqRN  = dirRN + File.separator + "rn" + File.separator;
          
            // Carrega as redes neurais geradas.
            NeuralNetwork moverRobo = NeuralNetwork.createFromFile(dirArqRN + "MoverRobo.nnet");
            NeuralNetwork buscarAlvo = NeuralNetwork.createFromFile(dirArqRN + "BuscarAlvo.nnet");
            
            FloatWA posicaoRobo = new FloatWA(3);    
            FloatWA orientacaoRobo = new FloatWA(3);         
            FloatWA posicaoDestino = new FloatWA(3);
            
            FloatWA velocidadeMotor = new FloatWA(2);
            FloatWA sensores = new FloatWA(6);           


            while(!nao_achou_alvo) {
                // Obter sensores do robo
                vrep.simxCallScriptFunction(conexaoVREPId, "K3_robot", vrep.sim_scripttype_childscript, "getSensors", null, null, null, null, null, sensores, null, null, vrep.simx_opmode_blocking);
                
                // Obter localização dorobo
                vrep.simxCallScriptFunction(conexaoVREPId, "K3_robot", vrep.sim_scripttype_childscript, "getPosition", null, null, null, null, null, posicaoRobo, null, null, vrep.simx_opmode_blocking);
                float posEixoRoboX = posicaoRobo.getArray()[0];
                float posEixoRoboY = posicaoRobo.getArray()[1];
               
                // obter a localizacao do destino
                vrep.simxCallScriptFunction(conexaoVREPId, "K3_robot", vrep.sim_scripttype_childscript, "getTargetPosition", null, null, null, null, null, posicaoDestino, null, null, vrep.simx_opmode_blocking);
                float posEixoAlvoX = posicaoDestino.getArray()[0];
                float posEixoAlvoY = posicaoDestino.getArray()[1];
                
                // Obter orientacao do robo
                vrep.simxCallScriptFunction(conexaoVREPId, "K3_robot", vrep.sim_scripttype_childscript, "getOrientation", null, null, null, null, null, orientacaoRobo, null, null, vrep.simx_opmode_blocking);
                float angleA = orientacaoRobo.getArray()[0];
                float angleB = orientacaoRobo.getArray()[1];
                float angle = ((angleA >= 0 ? 1f : -1f) * (float)Math.toRadians(90)) + angleB;
                double angle360 = Math.toDegrees(angle) > 0 ? Math.toDegrees(angle) : (Math.toDegrees(angle)*-1) + 180;
         
                //CALCULAR ANGULO PARA O TARGET
                 float distX = posEixoRoboX - posEixoAlvoX;
                 float distanceY = posEixoRoboY - posEixoAlvoY;
                 double distanceToTarget = Math.sqrt(Math.pow(distX,2) + Math.pow(distanceY,2));
                 double angleToTarget = Math.atan2(distanceY, distX);
                 double anguloDoAlvo = ((((Math.toDegrees(angleToTarget)-90)%360) + 360) % 360) ; // correcao para bater com o angulo do robo
                                
                float L = Math.min(sensores.getArray()[0], 0.2f) * 5;
                float LM = Math.min(sensores.getArray()[1], 0.2f) * 5;
                
                float LF = Math.min(sensores.getArray()[2], 0.2f) * 5;
                float RF = Math.min(sensores.getArray()[3], 0.2f) * 5;
                float F = (LF + RF) / 2;
                
                float RM = Math.min(sensores.getArray()[4], 0.2f) * 5;
                float R = Math.min(sensores.getArray()[5], 0.2f) * 5;
                
                float motorEsquerdo = 0;
                float motorDireito = 0; 
                
                System.out.println("L  = " + L); 
                System.out.println("LM = " + LM); 
                System.out.println("F  = " + F); 
                System.out.println("RM = " + RM); 
                System.out.println("R  = " + R); 
                System.out.println("L+LM+F+RM+R = " + (L+LM+F+RM+R)); 
          
                if (L+LM+F+RM+R < 5) {
                
                    moverRobo.setInput(L, LM, F, RM, R);
                    moverRobo.calculate();
                    double[] output = moverRobo.getOutput();
                    motorEsquerdo = (float)(output[0]*10)-5;
                    motorDireito = (float)(output[1]*10)-5;
                }
                else {
                    float distanciaNormalizada = (float)Math.min(distanceToTarget, 5)/5;
                    float dif = (float)Math.toRadians(anguloDoAlvo - ((((angle360-180)%360) + 360) % 360));
                    float diferencaAngulos = (float)Math.atan2(Math.sin(dif), Math.cos(dif));
                    float diferencaNormalizada = (float)((Math.toDegrees(diferencaAngulos) / 36) + 5)/10;
                    
                    buscarAlvo.setInput(distanciaNormalizada, diferencaNormalizada);                
                    buscarAlvo.calculate();
                    double[] output = buscarAlvo.getOutput();
                    motorEsquerdo = (float)(output[0]*8)-4;
                    motorDireito = (float)(output[1]*8)-4;
                }

               //  encontrou o destino
                if (distanceToTarget < 0.05){
                    motorEsquerdo = motorDireito = 0;
                }
                
                System.out.println("motorEsquerdo = " + motorEsquerdo);
                System.out.println("motorDireito = " + motorDireito);
                
                velocidadeMotor.getArray()[0] = motorEsquerdo; 
                velocidadeMotor.getArray()[1] = motorDireito; 
                vrep.simxCallScriptFunction(conexaoVREPId, "K3_robot", vrep.sim_scripttype_childscript, "setVelocity", null, velocidadeMotor, null, null, null, null, null, null, vrep.simx_opmode_blocking);
            }

            //close the connection to V-REP:   
            vrep.simxFinish(conexaoVREPId);
        }

    }
    
}
