#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <fstream>

using namespace argos;

class CSpcLoopFunctions : public CLoopFunctions
{

public:
   /**
    * Class constructor
    */
   CSpcLoopFunctions();

   /** 
    * Class destructor
    */
   virtual ~CSpcLoopFunctions();

   /**
    * Initializes the experiment.
    * It is executed once at the beginning of the experiment, i.e., when ARGoS is launched.
    * @param t_tree The parsed XML tree corresponding to the <loop_functions> section.
    */
   virtual void Init(TConfigurationNode &t_tree);

   virtual void Destroy();
   virtual void Reset();

   virtual void PreStep();

   virtual void PostStep();

   virtual void PostExperiment();

   CRandom::CRNG *m_pcRng;

private:
   Real ComputeObjectiveFunction();
   CVector2 RandomPointOnSquareArea();
   CVector2 RandomPointOnCirclePerimeter();
   bool IsOnSquareArea(CVector2 c_point);
   bool IsOnCirclePerimeter(CVector2 c_point);

private:
   Real m_fRadiusRobot;

   Real m_fSideSquare;
   Real m_fRadiusCircle;

   CVector2 m_cCoordCircleSpot;
   CVector2 m_cCoordSquareSpot;
   UInt32 m_unNumberPoints;

   Real m_fObjectiveFunction;
   Real m_fDoptA;
   Real m_fDoptP;

   std::string m_strOutFile;

   std::ofstream m_cOutFile;
};
