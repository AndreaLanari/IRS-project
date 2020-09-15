#include "spc_loop_functions.h"
#include <argos3/core/wrappers/lua/lua_controller.h>

/****************************************/
/***************************************/

CSpcLoopFunctions::CSpcLoopFunctions()
{ // empty target locations
  /* Initialize the target locations */
  m_fSideSquare = 0.6;
  m_fRadiusCircle = 0.3;
  m_fRadiusRobot = 0.04;

  m_cCoordCircleSpot = CVector2(-0.64, 0);
  m_cCoordSquareSpot = CVector2(0.64, 0);

  m_unNumberPoints = 1000;

  m_fObjectiveFunction = 0;
  m_fDoptA = 0.08;
  m_fDoptP = 0.06;
}

/****************************************/
/****************************************/

CSpcLoopFunctions::~CSpcLoopFunctions()
{
  /* Nothing to do */
}

/****************************************/
/****************************************/

void CSpcLoopFunctions::Init(TConfigurationNode &t_tree)
{
  m_pcRng = CRandom::CreateRNG("argos");
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

void CSpcLoopFunctions::Reset()
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

void CSpcLoopFunctions::Destroy()
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

void CSpcLoopFunctions::PreStep()
{
  /* Nothing to do */
}

///////////////////////////////
///////////////////////////////
void CSpcLoopFunctions::PostStep()
{
}

void CSpcLoopFunctions::PostExperiment()
{
  m_fObjectiveFunction = ComputeObjectiveFunction();
  m_cOutFile << GetSpace().GetSimulationClock() << "\t"
             << m_fObjectiveFunction
             << std::endl;
}

/****************************************/
/****************************************/
Real CSpcLoopFunctions::ComputeObjectiveFunction()
{
  CVector2 cRandomPoint;
  Real dA = 0, dP = 0;
  CSpace::TMapPerType mEpucks = GetSpace().GetEntitiesByType("e-puck");
  CVector2 cEpuckPosition(0, 0);
  Real fDistanceToRandomPoint = 0;

  // White square area
  for (UInt32 i = 0; i < m_unNumberPoints; i++)
  {

    Real fMinDistanceOnSquare = 0.85; // Correspond to the diagonal of the square area

    cRandomPoint = RandomPointOnSquareArea();

    for (CSpace::TMapPerType::iterator it = mEpucks.begin(); it != mEpucks.end(); ++it)
    {
      CEPuckEntity *pcEpuck = any_cast<CEPuckEntity *>((*it).second);
      cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                         pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      if (IsOnSquareArea(cEpuckPosition))
      {
        fDistanceToRandomPoint = (cRandomPoint - cEpuckPosition).Length();
        if (fDistanceToRandomPoint < fMinDistanceOnSquare)
        {
          fMinDistanceOnSquare = fDistanceToRandomPoint;
        }
      }
    }

    dA += fMinDistanceOnSquare;
  }
  dA /= m_unNumberPoints;

  // Black circle area
  for (UInt32 i = 0; i < m_unNumberPoints; ++i)
  {

    Real fMinDistanceOnCircle = 0.6; // Correspond to the diameter of the circular spot
    cRandomPoint = RandomPointOnCirclePerimeter();

    for (CSpace::TMapPerType::iterator it = mEpucks.begin(); it != mEpucks.end(); ++it)
    {
      CEPuckEntity *pcEpuck = any_cast<CEPuckEntity *>((*it).second);
      cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                         pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      if (IsOnCirclePerimeter(cEpuckPosition))
      {
        fDistanceToRandomPoint = (cRandomPoint - cEpuckPosition).Length();
        if (fDistanceToRandomPoint < fMinDistanceOnCircle)
        {
          fMinDistanceOnCircle = fDistanceToRandomPoint;
        }
      }
    }

    dP += fMinDistanceOnCircle;
  }
  dP /= m_unNumberPoints;
  Real performance = (dA / m_fDoptA) + (dP / m_fDoptP);
  return performance;
}

bool CSpcLoopFunctions::IsOnSquareArea(CVector2 c_point)
{
  CRange<Real> cRangeSquareX(m_cCoordSquareSpot.GetX() - m_fSideSquare / 2.0f, m_cCoordSquareSpot.GetX() + m_fSideSquare / 2.0f);
  CRange<Real> cRangeSquareY(m_cCoordSquareSpot.GetY() - m_fSideSquare / 2.0f, m_cCoordSquareSpot.GetY() + m_fSideSquare / 2.0f);

  if (cRangeSquareX.WithinMinBoundIncludedMaxBoundIncluded(c_point.GetX()) &&
      cRangeSquareY.WithinMinBoundIncludedMaxBoundIncluded(c_point.GetY()))
  {
    return true;
  }
  return false;
}

bool CSpcLoopFunctions::IsOnCirclePerimeter(CVector2 c_point)
{
  CRange<Real> cAcceptanceRange(m_fRadiusCircle - m_fRadiusRobot, m_fRadiusCircle + m_fRadiusRobot);
  Real fDistanceFromCenter = (c_point - m_cCoordCircleSpot).Length();
  if (cAcceptanceRange.WithinMinBoundIncludedMaxBoundIncluded(fDistanceFromCenter))
  {
    return true;
  }
  return false;
}

CVector2 CSpcLoopFunctions::RandomPointOnSquareArea()
{
  return CVector2(m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetX() - m_fSideSquare / 2.0f, m_cCoordSquareSpot.GetX() + m_fSideSquare / 2.0f)),
                  m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetY() - m_fSideSquare / 2.0f, m_cCoordSquareSpot.GetY() + m_fSideSquare / 2.0f)));
}

CVector2 CSpcLoopFunctions::RandomPointOnCirclePerimeter()
{
  CRadians cAngle = m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI));
  return CVector2(m_cCoordCircleSpot.GetX() + Cos(cAngle) * m_fRadiusCircle, m_cCoordCircleSpot.GetY() + Sin(cAngle) * m_fRadiusCircle);
}

/****************************************/
/****************************************/

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CSpcLoopFunctions, "spc_loop_functions");
