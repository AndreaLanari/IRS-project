#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <fstream>

using namespace argos;

class CScaLoopFunctions : public CLoopFunctions
{

public:
   /**
    * Class constructor
    */
   CScaLoopFunctions();

   /** 
    * Class destructor
    */
   virtual ~CScaLoopFunctions();

   /**
    * Initializes the experiment.
    * It is executed once at the beginning of the experiment, i.e., when ARGoS is launched.
    * @param t_tree The parsed XML tree corresponding to the <loop_functions> section.
    */
   virtual void Init(TConfigurationNode &t_tree);

   /**
    * Resets the experiment to the state it was right after Init() was called.
    * It is executed every time you press the 'reset' button in the GUI.
    */
   virtual void Reset();

   /**
    * Undoes whatever Init() did.
    * It is executed once when ARGoS has finished the experiment.
    */
   virtual void Destroy();

   /**
    * Performs actions right before a simulation step is executed.
    */
   virtual void PreStep();

   /**
    * Performs actions right after a simulation step is executed.
    */
   virtual void PostStep();

   virtual void PostExperiment();

private:
   bool IsInShelter(CVector2 &c_position);

private:
   Real m_fSpotRadius;
   Real m_fWidthShelter;
   Real m_fHeightShelter;
   CVector2 m_cPositionShelter;
   Real m_fObjectiveFunction;

   /**
    * The path of the output file.
    */
   std::string m_strOutFile;

   /**
    * The stream associated to the output file.
    */
   std::ofstream m_cOutFile;
};
