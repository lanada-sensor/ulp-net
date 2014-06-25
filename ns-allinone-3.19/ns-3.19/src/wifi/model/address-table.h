#ifndef ADDRESS_TABLE_H
#define ADDRESS_TABLE_H

#include <iostream>
#include <stdio.h>
#include <map>
#include "ns3/address.h"

#define _DEBUG_ADDRESS_TABLE 0
//#include "ns3/node.h"


//using namespace std;


namespace ns3 {

class AddressTable {
public:
	AddressTable(){
		}
//	~AddressTable();
	bool AddAddress(Address & main, Address & ulp)
	{
		Address mainAddress(main);
		Address ulpAddress(ulp);
		this->addressTable.insert(std::pair<Address, Address>(mainAddress,ulpAddress));
		return true;
	}
	Address GetUlpAddress(Address main)
	{
		std::map<Address, Address>::iterator it;
		for (it = addressTable.begin (); it!= addressTable.end (); it++)
		{
			if((*it).first == main){
#if _DEBUG_ADDRESS_TABLE
				std::cout<<"matched!\n";
#endif
				return (*it).second;

			}
		}
		return main;
	}
	Address GetMainAddress(Address ulp)
	{
		std::map<Address, Address>::iterator it;
		for (it = addressTable.begin (); it!= addressTable.end (); it++)
		{
			if((*it).second == ulp){
#if _DEBUG_ADDRESS_TABLE
				std::cout<<"matched!\n";
#endif
				return (*it).first;

			}
		}
		return ulp;
	}
	int GetSize(){
		return this->addressTable.size();
	}


private:
	std::map<Address, Address> addressTable;
};

}

#endif /* ADDRESS_TABLE_H */
