/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       FGPropulsion.h
 Author:       Jon S. Berndt
 Date started: 08/20/00

 ------------- Copyright (C) 1999  Jon S. Berndt (jon@jsbsim.org) -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 Place - Suite 330, Boston, MA  02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be found on
 the world wide web at http://www.gnu.org.

HISTORY
--------------------------------------------------------------------------------
08/20/00   JSB   Created

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SENTRY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef FGPROPULSION_H
#define FGPROPULSION_H

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <vector>
#include <iosfwd>

#include "FGModel.h"
#include "propulsion/FGEngine.h"
#include "math/FGMatrix33.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DEFINITIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define ID_PROPULSION "$Id: FGPropulsion.h,v 1.35 2015/01/07 23:22:59 dpculp Exp $"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FORWARD DECLARATIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

namespace JSBSim {

class FGTank;
class FGEngine;

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DOCUMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/** Propulsion management class.
    The Propulsion class is the container for the entire propulsion system, which is
    comprised of engines, and tanks. Once the Propulsion class gets the config file,
    it reads in the \<propulsion> section. Then:

    -# The appropriate engine type instance is created
    -# At least one tank object is created, and is linked to an engine.

    At Run time each engine's Calculate() method is called.

    <h3>Configuration File Format:</h3>

  @code
    <propulsion>
        <engine file="{string}">
          ... see FGEngine, FGThruster, and class for engine type ...
        </engine>
        ... more engines ...
        <tank type="{FUEL | OXIDIZER}"> 
          ... see FGTank ...
        </tank>
        ... more tanks ...
        <dump-rate unit="{LBS/MIN | KG/MIN}"> {number} </dump-rate>
        <refuel-rate unit="{LBS/MIN | KG/MIN}"> {number} </refuel-rate>
    </propulsion>
  @endcode

    @author Jon S. Berndt
    @version $Id: FGPropulsion.h,v 1.35 2015/01/07 23:22:59 dpculp Exp $
    @see
    FGEngine
    FGTank
*/

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DECLARATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGPropulsion : public FGModel
{
public:
  /// Constructor
  FGPropulsion(FGFDMExec*);
  /// Destructor
  ~FGPropulsion();

  /** Executes the propulsion model.
      The initial plan for the FGPropulsion class calls for Run() to be executed,
      calculating the power available from the engine.
      Can pass in a value indicating if the executive is directing the simulation to Hold.
      @param Holding if true, the executive has been directed to hold the sim from 
                     advancing time. Some models may ignore this flag, such as the Input
                     model, which may need to be active to listen on a socket for the
                     "Resume" command to be given.
      @return false if no error */
  bool Run(bool Holding);

  bool InitModel(void);

  /** Loads the propulsion system (engine[s] and tank[s]).
      Characteristics of the propulsion system are read in from the config file.
      @param el pointer to an XML element that contains the engine information.
      @return true if successfully loaded, otherwise false */
  bool Load(Element* el);

  /// Retrieves the number of engines defined for the aircraft.
  virtual unsigned int GetNumEngines(void) const {return (unsigned int)Engines.size();}

  /** Retrieves an engine object pointer from the list of engines.
      @param index the engine index within the vector container
      @return the address of the specific engine, or zero if no such engine is
              available */
  virtual FGEngine* GetEngine(unsigned int index) const {
                      if (index < Engines.size()) return Engines[index];
                      else                        return 0L;      }

  /// Retrieves the number of tanks defined for the aircraft.
  virtual unsigned int GetNumTanks(void) const { return (unsigned int)Tanks.size(); }

  /** Retrieves a tank object pointer from the list of tanks.
      @param index the tank index within the vector container
      @return the address of the specific tank, or zero if no such tank is
              available */
  virtual FGTank* GetTank(unsigned int index) const {
                      if (index < Tanks.size()) return Tanks[index];
                      else                      return 0L;        }

  /** Returns the number of fuel tanks currently actively supplying fuel */
  virtual int GetnumSelectedFuelTanks(void) const { return numSelectedFuelTanks; }

  /** Returns the number of oxidizer tanks currently actively supplying oxidizer */
  virtual int GetnumSelectedOxiTanks(void) const { return numSelectedOxiTanks; }

  /** Loops the engines until thrust output steady (used for trimming) */
  virtual bool GetSteadyState(void);

  /** Sets up the engines as running */
  virtual void InitRunning(int n);

  virtual std::string GetPropulsionStrings(const std::string& delimiter) const;
  virtual std::string GetPropulsionValues(const std::string& delimiter) const;
  virtual std::string GetPropulsionTankReport();

  virtual const FGColumnVector3& GetForces(void) const { return vForces; }
  virtual double GetForces(int n) const { return vForces(n); }
  virtual const FGColumnVector3& GetMoments(void) const { return vMoments; }
  virtual double GetMoments(int n) const { return vMoments(n); }

  virtual bool GetRefuel(void) const { return refuel; }
  virtual void SetRefuel(bool setting) { refuel = setting; }
  virtual bool GetFuelDump(void) const { return dump; }
  virtual void SetFuelDump(bool setting) { dump = setting; }
  virtual double Transfer(int source, int target, double amount);
  virtual void DoRefuel(double time_slice);
  virtual void DumpFuel(double time_slice);

  virtual const FGColumnVector3& GetTanksMoment(void);
  virtual double GetTanksWeight(void) const;

  virtual std::string FindFullPathName(const std::string& filename) const;
  virtual inline int GetActiveEngine(void) const { return ActiveEngine; }
  virtual inline bool GetFuelFreeze(void) const { return FuelFreeze; }
  virtual double GetTotalFuelQuantity(void) const { return TotalFuelQuantity; }

  virtual void SetMagnetos(int setting);
  virtual void SetStarter(int setting);
  virtual void SetCutoff(int setting = 0);
  virtual void SetActiveEngine(int engine);
  virtual void SetFuelFreeze(bool f);
  virtual const FGMatrix33& CalculateTankInertias(void);

  struct FGEngine::Inputs in;

private:
  std::vector <FGEngine*>   Engines;
  std::vector <FGTank*>     Tanks;
  unsigned int numSelectedFuelTanks;
  unsigned int numSelectedOxiTanks;
  unsigned int numFuelTanks;
  unsigned int numOxiTanks;
  unsigned int numEngines;
  unsigned int numTanks;
  int ActiveEngine;
  FGColumnVector3 vForces;
  FGColumnVector3 vMoments;
  FGColumnVector3 vTankXYZ;
  FGColumnVector3 vXYZtank_arm;
  FGMatrix33 tankJ;
  bool refuel;
  bool dump;
  bool FuelFreeze;
  double TotalFuelQuantity;
  double DumpRate;
  double RefuelRate;
  bool IsBound;
  bool HavePistonEngine;
  bool HaveTurbineEngine;
  bool HaveTurboPropEngine;
  bool HaveRocketEngine;
  bool HaveElectricEngine;
  void ConsumeFuel(FGEngine* engine);

  bool ReadingEngine;

  void bind();
  void Debug(int from);
};
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#endif

