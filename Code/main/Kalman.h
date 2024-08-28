/* FILTRO KALMAN */

// Si no esta definidala clase Kalman.h, la creamos:
#ifndef _Kalman_h
#define _Kalman_h

class Kalman {
public:
    Kalman() {
    // Valor por defecto, pueden cambiarse al gusto.
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;

        angle = 0; // Reseteo el ángulo.
        bias = 0;  // Reseteo la tendencia.
        
    // Asumo una tendencia=0 y conozco el ángulo de inicio (setAngle)por tanto la matriz covariante de error queda asi:
        P[0][0] = 0; 
        P[0][1] = 0; 
        P[1][0] = 0;
        P[1][1] = 0;
    };
    // El ángulo debería ir en grados(º), el giro en grados por segundo(º/s) y la variacion de tiempo en segundos(s).
    double getAngle(double newAngle, double newRate, double dt) {
        // Módulo del filtro Kalman.
        
        // Ecuaciones de actualización de tiempo de un filtro Kalman discreto - Time Update ("Predict").
        /* Paso 1 */
        rate = newRate - bias;
        angle += dt * rate;
        
        // Matriz covariante de actualización de error - Project the error covariance ahead.
        /* Paso 2 */
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Ecuaciones de actuaización de medida de un filtro Kalman discreto - Measurement Update ("Correct").
        
        // Cálculo de la ganancia Kalman - Compute the Kalman gain.
        /* Paso 4 */
        S = P[0][0] + R_measure;
        
        /* Paso 5 */
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
        
        // Cálculo el angulo y la tendencia - Update estimate with measurement zk (newAngle).
        /* Paso 3 */
        y = newAngle - angle;
        
        /* Paso 6 */
        angle += K[0] * y;
        bias += K[1] * y;
        
        // Cálculo de la matriz covariante de la estimación del error - Update the error covariance.
        /* Paso 7 */
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return angle; // Devuelve el ángulo filtrado
    };
    
    void setAngle(double newAngle) { angle = newAngle; }; // Esta linea se usa para indicar el angulo, deberia usarse para inicial el angulo inicial. 
    double getRate() { return rate; }; // Devuelve el giro filtrado.

    /* Estas líneas fueron usadas para calibrar el filtro Kalman */ 
    void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
    void setRmeasure(double newR_measure) { R_measure = newR_measure; };

    double getQangle() { return Q_angle; };
    double getQbias() { return Q_bias; };
    double getRmeasure() { return R_measure; };

private:
    /* Variables del filtro Kalman */
    double Q_angle;   // Procesa la variación de ruido para el acelerómetro.
    double Q_bias;    // Procesa la variacion de ruido para la tendencia del giroscopio.
    double R_measure; // Medición de la variación de ruido (Para ser exactos es la variación de la medida del ruido).

    double angle;     // El ángulo calculado por el filtro Kalman (Parte del vector de posición 2x1 [state vector]).
    double bias;      // La tendencia del giroscopio calculado por el filtro Kalman (Parte del vector de posición 2x1 [state vector]).
    double rate;      // Giro filtrado calculado a partir del giro y la tendencia calculada (Tienes que llamar a getAngle para actualizar el giro).
 
    double P[2][2];   // Matriz covariante del error(Es una matriz 2x2).
    double K[2];      // Ganancia Kalman (Es un vector 2x1).
    double y;         // Diferencia del angulo.
    double S;         // Error estimado.
};

#endif
