
/**
 * 
 * The Rubber band project: Real time finite-element based haptic rendering
 * Interfacing with ARDUINO DUE, BLDC Motor, Encoder
 * Based on SimpleFOC library for BLDC control
 * FE implementation based on MATLAB code by Morten D. Pedersen, NTNU
 * By Øystein Bjelland, CPS Lab, NTNU
 */

#include <math.h> // FOR SIMPLE MATH OPERATIONS
#include <BasicLinearAlgebra.h> // FOR MATRIX OPERATIONS: https://github.com/tomstewart89/BasicLinearAlgebra 
#include <ElementStorage.h> // For BLA library


template<int N>
struct Deformation
{
  double force;
  BLA::Matrix<N> deformedShape;
};

template<int N>
class RubberBand
{
public:

  RubberBand() 
  {
    double k1 = 5.000;
    double k2 = 3.333;
    double k3 = -1.6667;
    double zero = 0;

    K.Fill(0.0);
    K(0, 0) = k1;
    K(0, 1) = k3;
    K(N-1, N-1) = k1;
    K(N-1, N-2) = k3;
  
    for(auto i = 1; i < N -1; i++)
    {
      K(i, i-1) = k3;
      K(i, i) = k2;
      K(i, i+1) = k3;     
    }
  }

  void setup()
  {
   
    Serial.println("Starting matrix initialization");
    
    xs.Fill(0); // Fill vector containing node positions     
    for (int i = 0; i<= N; i++){
      xs(i) = a + ((b-a)/N)*i;   
    }
    
    xm.Fill(0); // Fill vector containing element midpoint positions
    for (int j = 0; j<= N-1; j++){
      xm(j) = (xs(j) + xs(j+1))/2;
    }
    Serial.println("Matrices initialized");
  }
  
  Deformation<N> loop(float xhDeg)
  {

    BLA::Matrix<1> XH = {xhDeg};
    auto xPen = XH - xRubberband;

    //Collision detection start************************************************************ 
    //Find y-position of tool (adjacent nodes) NB! yh is so far constant.
    
    for (int i = 0; i<= N-1; i++){ 
        if(yh >= xs(i)){
           prevElementIndex = i;
            if (yh <= xs(i+1)){
              nextElementIndex = i+1;
            }
        }
    }
  
    BLA::Matrix<N> cX; 
    cX.Fill(0);
    cX(prevElementIndex) = (xm(nextElementIndex) - yh) / (xm(nextElementIndex) - xm(prevElementIndex));
    cX(nextElementIndex) = (yh - xm(prevElementIndex)) / (xm(nextElementIndex) - xm(prevElementIndex));

    auto K_decomp = K; //LU Decompose will destroy stiffness matrix K, so we make a copy
    auto decomp = LUDecompose(K_decomp);
  
    BLA::Matrix<N> K_NEW = LUSolve(decomp, cX);   //Solving K*k_new = cX using LU decomposition. Equivalent to 'linsolve(K, cX)' in Matlab.  
    BLA::Matrix<1> KK = ~cX * K_NEW;
    BLA::Matrix<1> KK_INV = KK;
    Invert(KK_INV);  //Invert matrix. Potentially computationally expensive(!)
    
    BLA::Matrix<1> Fc = xPen*KK_INV;   // Could be done with scalars really.
      Deformation<N> state;
      state.force = Fc(0);

    BLA::Matrix<N> distributedForce = cX*Fc; // 
    state.deformedShape = LUSolve(decomp, distributedForce);   // Deformed mesh of rubber band
      return state;
  }

  BLA::Matrix<N+1> getXS(){
    return xs;
  }

  BLA::Matrix<N> getXM(){
    return xm;
  }

private:
  BLA::Matrix<1> xPen;                // Instrument penetration [deg]
  BLA::Matrix<1> xRubberband = {20};  // Position of rubberband [deg] (posititon where tool meets rubber band)
  double yh = 1.5;                      // y-position of tool. Position of tool along the rubber band. (so far, this is only constant)

  //Rubberband variables
  double a = 0.5;  //Left fixpoint [m]
  double b = 2;    //Right fixpoint [m]
  double T = 1;    //Tension 
  
  BLA::Matrix<N+1> xs;  //Define vector xs containing position of nodes
  BLA::Matrix<N> xm;  //Define vector xm containing position of element midpoints
  
  BLA::Matrix<N,N> K;
  
  int prevElementIndex = 0;
  int nextElementIndex = 0;
 
};
