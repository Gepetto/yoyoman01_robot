// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <stdexcept>
#include <sot/core/debug.hh>

#include <dynamic-graph/factory.h>
#include "dynamic-yoyoman01.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::sot;


namespace dynamicgraph
{
  namespace sot
  {
    namespace yoyoman01
    {

      DynamicYoyoman01::DynamicYoyoman01 (const std::string & name)
	: Dynamic (name, false)
      {
	sotDEBUGIN(15);
	DynamicYoyoman01::buildModel ();
	sotDEBUGOUT(15);
      }

      DynamicYoyoman01::~Dynamicyoyoman01 ()
      {
	sotDEBUGINOUT(5);
	return;
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicYoyoman01,"DynamicYoyoman01");
    } // end of namespace yoyoman01.
  } // end of namespace sot.
} // end of namespace dynamicgraph.
