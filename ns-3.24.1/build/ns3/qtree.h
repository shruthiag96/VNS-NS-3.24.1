/***************************************************************************
 *   Copyright (C) 2012 Ricardo Fernandes - All Rights Reserved            *
 *   Email: rjf@dcc.fc.up.pt or rikardojfernandez@gmail.com                *
 *                                                                         *
 *   This file is part of the project that integrates VNS and NS-3         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef VNS_NSQTREE_H
#define VNS_NSQTREE_H

#include "ns3/vector.h"
#include <list>

namespace ns3 {

class QTreeNode;

class QTreeItem {
public:
	QTreeNode* parent;
	virtual ~QTreeItem(){};

public:
	virtual Vector getPoint() = 0;
	friend class QTree;
};

class QTreeNode {

public:

	QTreeNode* childs[4];
	QTreeNode* parent;
	std::list< QTreeItem* > items;
	bool isLeaf;
	double minX;
	double minY;
	double maxX;
	double maxY;

	QTreeNode(QTreeNode* p,double minX,double minY,double maxX,double maxY);
	~QTreeNode();

public:
	bool contains(const Vector& point);

	friend class QTree;
};

class QTreeVisitor {
public:
	virtual ~QTreeVisitor(){};
	virtual bool accept(QTreeNode* node) = 0;
	virtual bool accept(QTreeItem* item) = 0;
	virtual bool acceptParent(QTreeNode*){ return false; };
	virtual void visit(QTreeItem* item) = 0;
	double minX;
	double minY;
	double maxX;
	double maxY;
};

class QTree {
private:
	void insertInternal(QTreeItem* item , QTreeNode* n, int level);
	void insertUp(QTreeItem* item , QTreeNode* n, int level);
	void updatePositions(QTreeNode* n, std::list< QTreeItem* >& items, int level);
	void applyVisitor(QTreeNode* n, QTreeVisitor& visitor);
	bool isToClean(QTreeNode* n);

public:
	QTree(double minX, double minY, double maxX,double maxY);
	~QTree();
	void addItem(QTreeItem* item);

	void update();

	void clear();
	inline int size() const { return m_size; };
	void remove(QTreeItem* item);

	void applyVisitor(QTreeVisitor& visitor);
	void applyVisitorBottomUp(QTreeItem* item, QTreeVisitor& visitor);

private:
	QTreeNode* root;
	int m_size;
	int maxLevels;

	friend class QTreeNode;
	friend class QTreeItem;
};

}

#endif
