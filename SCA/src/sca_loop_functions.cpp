#include "sca_loop_functions.h"
#include <argos3/core/wrappers/lua/lua_controller.h>

/****************************************/
/***************************************/

CScaLoopFunctions::CScaLoopFunctions()
{ // empty target locations
  /* Initialize the target locations */
  m_fObjectiveFunction = 0;
  m_fWidthShelter = 0.6;
  m_fHeightShelter = 0.15;
  m_cPositionShelter = CVector2(0, 0);
}

/****************************************/
/****************************************/

CScaLoopFunctions::~CScaLoopFunctions()
{
  /* Nothing to do */
}

/****************************************/
/****************************************/

void CScaLoopFunctions::Init(TConfigurationNode &t_tree)
{
  /* Get output file name from XML tree */
  GetNodeAttribute(t_tree, "output", m_strOutFile);
  /* Open the file for text writing */
  m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (m_cOutFile.fail())
  {
    THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
  }
  /* Write a header line */
  m_cOutFile << "# Clock\tPoints" << std::endl;
}

/****************************************/
/****************************************/

void CScaLoopFunctions::Reset()
{
  /* Set to false the target flags */
  /* Close the output file */
  m_cOutFile.close();
  if (m_cOutFile.fail())
  {
    THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\"");
  }
  /* Open the file for text writing */
  m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
  if (m_cOutFile.fail())
  {
    THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
  }
  /* Write a header line */
  m_cOutFile << "# Clock\tPoints" << std::endl;
}

/****************************************/
/****************************************/

void CScaLoopFunctions::Destroy()
{
  /* Close the output file */
  m_cOutFile.close();
  if (m_cOutFile.fail())
  {
    THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\"");
  }
}

/****************************************/
/****************************************/

void CScaLoopFunctions::PreStep()
{
  /* Nothing to do */
}

///////////////////////////////
///////////////////////////////
void CScaLoopFunctions::PostStep()
{
  CSpace::TMapPerType &tEPuckMap = GetSpace().GetEntitiesByType("e-puck");
  CVector2 cEPuckPosition(0, 0);

  for (CSpace::TMapPerType::iterator it = tEPuckMap.begin(); it != tEPuckMap.end(); ++it)
  {
    CEPuckEntity *pcEPuck = any_cast<CEPuckEntity *>(it->second);
    cEPuckPosition.Set(pcEPuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEPuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    if (IsInShelter(cEPuckPosition))
    {
      m_fObjectiveFunction += 1;
    }
  }
}

void CScaLoopFunctions ::PostExperiment()
{
  m_cOutFile << GetSpace().GetSimulationClock() << "\t"
             << m_fObjectiveFunction
             << std::endl;
}

/****************************************/
/****************************************/
bool CScaLoopFunctions::IsInShelter(CVector2 &c_position)
{
  Real fMaximalXCoord = m_fWidthShelter / 2;
  Real fMaximalYCoord = (m_fHeightShelter / 2) + m_cPositionShelter.GetY();
  if (c_position.GetX() > -fMaximalXCoord && c_position.GetX() < fMaximalXCoord)
  {
    if (c_position.GetY() > -fMaximalYCoord && c_position.GetY() < fMaximalYCoord)
    {
      return true;
    }
  }
  return false;
}

/****************************************/
/****************************************/

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CScaLoopFunctions, "sca_loop_functions");
