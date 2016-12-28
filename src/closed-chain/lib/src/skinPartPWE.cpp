#include "iCub/periPersonalSpace/skinPartPWE.h"

using namespace        yarp::sig;
using namespace iCub::skinDynLib;
using namespace              std;

/****************************************************************/
/* SKINPART TAXEL PWE WRAPPER
*****************************************************************/
    skinPartPWE::skinPartPWE(const std::string &_modality) :
                             skinPart(), modality(_modality)
    {
        clearTaxels();
    }

    skinPartPWE::skinPartPWE(const skinPartPWE &_spwe)
    {
        *this = _spwe;
    }

    skinPartPWE & skinPartPWE::operator=(const skinPartPWE &spw)
    {
        if (this == &spw)
        {
            return *this;
        }

        iCub::skinDynLib::skinPart::operator=(spw);

        modality  = spw.modality;

        clearTaxels();
        if (modality=="1D")
        {
            for (std::vector<Taxel*>::const_iterator it = spw.taxels.begin();
                 it != spw.taxels.end(); ++it)
            {
                taxels.push_back(new TaxelPWE1D(*(*it)));
            }
        }
        else if (modality=="2D")
        {
            for (std::vector<Taxel*>::const_iterator it = spw.taxels.begin();
                 it != spw.taxels.end(); ++it)
            {
                taxels.push_back(new TaxelPWE2D(*(*it)));
            }
        }

        return *this;
    }

    void skinPartPWE::print(int verbosity)
    {
        iCub::skinDynLib::skinPart::print(verbosity);
        for (size_t i = 0; i < taxels.size(); i++)
            taxels[i]->print(verbosity);
        yDebug("**********\n");
    }

    string skinPartPWE::toString(int precision)
    {
        stringstream res(iCub::skinDynLib::skinPart::toString(precision));
        for (size_t i = 0; i < taxels.size(); i++)
            res << taxels[i]->toString(precision);
        res << "**********\n";
        return res.str();
    }

    skinPartPWE::~skinPartPWE()
    {
        // printf("Taxelsize %lu %i\n", taxels.size(), get_taxelSize());
        // int i=0;

        // while(!taxels.empty())
        // {
        //     printf("i %i\n", i); i++;
        //     if (taxels.back())
        //     {
        //         delete taxels.back();
        //     }
        //     taxels.pop_back();
        // }
    }

// empty line to make gcc happy
