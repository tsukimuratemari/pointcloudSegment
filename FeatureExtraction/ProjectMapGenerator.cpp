#include "pch.h"
#include "ProjectMapGenerator.h"

void CProjectMapGenerator::generate(int vWidth, int vHeight)
{
	_ASSERTE(!m_pCloud->empty() && vWidth >= 0 && vHeight >= 0 && (vWidth + vHeight) > 0);
	m_Map.setSize(vWidth, vHeight);
	if (!m_Box.isValid())
	{
		CAABBEstimation AABBEstimation(m_pCloud);
		m_Box = AABBEstimation.compute();
	}

	for (auto& Point : *m_pCloud)
	{
		auto Offset = computeOffset(Point);
		m_Map.setValueAt(Point, Offset[0], Offset[1]);
	}
}

Eigen::Vector2i CProjectMapGenerator::computeOffset(const PointT& vPoint) const
{
	int OffsetX = (vPoint.getVector3fMap()[0] - m_Box._Min[0]) / (m_Box._Max[0] - m_Box._Min[0]) * m_Map.getWidth();
	int OffsetY = (vPoint.getVector3fMap()[1] - m_Box._Min[1]) / (m_Box._Max[1] - m_Box._Min[1]) * m_Map.getHeight();

	if (m_Box._Max[0] == m_Box._Min[0]) OffsetX = 0;
	if (m_Box._Max[1] == m_Box._Min[1]) OffsetY = 0;

	if (OffsetX == m_Map.getWidth()) OffsetX--;
	if (OffsetY == m_Map.getHeight()) OffsetY--;

	return Eigen::Vector2i(OffsetX, OffsetY);
}