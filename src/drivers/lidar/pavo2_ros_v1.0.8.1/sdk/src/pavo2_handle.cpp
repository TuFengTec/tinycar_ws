#include "pavo2_handle.h"
#include <iostream>
pavo2::HandlePacket::HandlePacket()
{
	m_pMADataFilter = new MADataFilter();
}

pavo2::HandlePacket::~HandlePacket()
{
	if(m_pMADataFilter)
	{
		delete m_pMADataFilter;
		m_pMADataFilter = nullptr;
	}
}

bool pavo2::HandlePacket::enable_tail_filter(uint16_t method)
{
	if(m_pMADataFilter)
	{
		m_pMADataFilter->EnableTailFilter(method);
		return true;
	}
	return false;
}

bool pavo2::HandlePacket::enable_front_filter(uint16_t iLevel)
{
	if(m_pMADataFilter)
	{
		m_pMADataFilter->EnableFrontFilter(iLevel);
		return true;
	}
	return false;
}
