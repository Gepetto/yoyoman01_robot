// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of sot-romeo.
// sot-romeo is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// sot-yoyoman01 is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-yoyoman01. If not, see <http://www.gnu.org/licenses/>.

#ifndef SOT_YOYOMAN01_DYNAMIC_HH
# define SOT_YOYOMAN01_DYNAMIC_HH
# include <sot-dynamic/dynamic.h>

# if defined (WIN32)
#   if defined (dynamic_yoyoman01_EXPORTS)
#     define DYNAMICYOYOMAN01_EXPORT __declspec(dllexport)
#   else
#     define DYNAMICYOYOMAN01_EXPORT __declspec(dllimport)
#   endif
# else
#   define DYNAMICYOYOMAN01_EXPORT
# endif

namespace dynamicgraph
{
  namespace sot
  {
    namespace yoyoman01
    {
      class DYNAMICYOYOMAN01_EXPORT DynamicYoyoman01 : public Dynamic
      {
	DYNAMIC_GRAPH_ENTITY_DECL ();
      public:
	explicit DynamicYoyoman01 (const std::string& name);
	virtual ~DynamicYoyoman01 ();
      };
    } // end of namespace yoyoman01.
  } // end of namespace sot.
} // end of namespace dynamicgraph.


#endif //! SOT_YOYOMAN01_DYNAMIC_HH
