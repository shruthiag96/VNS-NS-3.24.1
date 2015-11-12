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

#include "qtree.h"
#include <stdio.h>
#include <stdlib.h>
#include <queue>

namespace ns3 {

bool QTreeNode::contains(const Vector& pt){
	return pt.x>=minX && pt.x<=maxX && pt.y>=minY && pt.y<=maxY;
}

QTreeNode::QTreeNode(QTreeNode* p,double minX,double minY,double maxX,double maxY):minX(minX),minY(minY),maxX(maxX),maxY(maxY){
	parent = p;
	for(int i=0;i<4;i++){
		childs[i] = 0;
	}
	isLeaf = true;
}

QTreeNode::~QTreeNode(){
	for(int i=0;i<4;i++){
		if( childs[i] != 0 ){
			delete childs[i];
			childs[i] = 0;
		}
	}
	items.clear();
	parent = 0;
}

QTree::QTree(double minX, double minY, double maxX,double maxY){
	root = new QTreeNode(0,minX,minY,maxX,maxY);
	m_size = 0;
	maxLevels = 7;
}

QTree::~QTree(){
	delete root;
	m_size = 0;
}


void QTree::addItem(QTreeItem* item){
	if( root->contains( item->getPoint() ) ){
		insertInternal(item,root,0);
	}else{
		fprintf(stderr,"Vehicle is outside QTree extents.\n");
		exit(0);
	}
}

void QTree::insertInternal(QTreeItem* item , QTreeNode* n, int level){
	if( n->isLeaf == true ){
		if( n->items.size() == 0 || level >= maxLevels){
			item->parent = n;
			n->items.push_back( item );
			m_size++;
			return;
		}else{
			n->isLeaf = false;
			n->childs[0] = new QTreeNode( n, n->minX , (n->maxY+n->minY)*0.5 , (n->maxX+n->minX)*0.5 , n->maxY );
			n->childs[1] = new QTreeNode( n, (n->maxX+n->minX)*0.5 , (n->maxY+n->minY)*0.5 , n->maxX , n->maxY );
			n->childs[2] = new QTreeNode( n, n->minX , n->minY , (n->maxX+n->minX)*0.5 , (n->maxY+n->minY)*0.5 );
			n->childs[3] = new QTreeNode( n, (n->maxX+n->minX)*0.5 , n->minY , n->maxX , (n->maxY+n->minY)*0.5 );

			while( n->items.size() != 0 ){
				QTreeItem* leaf = n->items.back();
				leaf->parent = 0;
				n->items.pop_back();
				m_size--;
				for(int i=0;i<4;i++){
					QTreeNode* child = n->childs[i];
					if( child->contains( leaf->getPoint() ) ){
						insertInternal( leaf, child, level+1 );
						break;
					}
				}
			}
		}
	}
	for(int i=0;i<4;i++){
		QTreeNode* child = n->childs[i];
		if( child->contains( item->getPoint() ) ){
			insertInternal( item , child, level+1 );
			return;
		}
	}
}


void QTree::update(){
	std::list< QTreeItem* > items;
	updatePositions(root, items, 0);
}

void QTree::updatePositions(QTreeNode* n, std::list< QTreeItem* >& leafs, int level){
	if (n->isLeaf) {
		if(n->items.size() > 0) {
			std::list< QTreeItem* >::iterator it = n->items.begin();
			while( it != n->items.end() ){
				if( !n->contains( (*it)->getPoint() ) ) {
					QTreeItem* item = (*it);
					item->parent = 0;
					leafs.push_back( item );
					it = n->items.erase(it);
					m_size--;
				}else{
					++it;
				}
			}
		}
		return;
	} else {
		for(int i=0;i<4;i++){
			QTreeNode* child = n->childs[i];
			if( child != 0 ){
				updatePositions(child, leafs, level+1);
			}
		}
		if( isToClean(n) ){
			for(int i=0;i<4;i++){
				delete n->childs[i];
				n->childs[i] = 0;
			}
			n->isLeaf = true;
		}
		std::list< QTreeItem* >::iterator it = leafs.begin();
		while( it != leafs.end() ){
			if( n->contains( (*it)->getPoint() ) ) {
				QTreeItem* leaf = *it;
				it = leafs.erase(it);
				insertInternal( leaf , n , level );
			}else{
				it++;
			}
		}
	}
}

void QTree::insertUp(QTreeItem* item , QTreeNode* n, int level){
	// clean childs if that's the case

	if( isToClean(n) ){
		for(int i=0;i<4;i++){
			delete n->childs[i];
			n->childs[i] = 0;
		}
		n->isLeaf = true;
	}
	if( n->contains( item->getPoint() ) ){
		insertInternal( item, n, level);
	}else{
		if(n->parent != 0){
			insertUp(item, n->parent, level-1);
		}else{

		}
	}
}

bool QTree::isToClean(QTreeNode* n){
	for(int i=0;i<4;i++){
		QTreeNode* child = n->childs[i];
		if( child != 0 ){
			if( child->isLeaf == false ){
				return false;
			}
			if( child->items.size() > 0){
				return false;
			}
		}
	}
	return true;
}

/*
void QTree::applyVisitor(QTreeNode* n, QTreeVisitor& visitor){
	std::list<QTreeNode*> queue;
	queue.push_back(n);
	while( queue.size() > 0 ){
		QTreeNode* n = queue.back();
		queue.pop_back();
		if(n==0) continue;
		if(n->isLeaf){
			std::list< QTreeItem* >::iterator it;
			for(it=n->items.begin(); it != n->items.end(); ++it){
				if ( visitor.accept( *it ) ){
					visitor.visit( *it );
				}
			}
		}else{
			for(int i=0;i<4;i++){
				QTreeNode* child = n->childs[i];
				if ( visitor.accept( child ) ){
					queue.push_back( child );
				}
			}
		}
	}
}

*/
void QTree::applyVisitor(QTreeNode* n, QTreeVisitor& visitor){
	if(n==0) return;
	if(n->isLeaf){
		std::list< QTreeItem* >::iterator it;
		for(it=n->items.begin(); it != n->items.end(); ++it){
			if ( visitor.accept( *it ) ){
				visitor.visit( *it );
			}
		}
	}else{
		for(int i=0;i<4;i++){
			QTreeNode* child = n->childs[i];
			if ( visitor.accept( child ) ){
				applyVisitor(child,visitor);
			}
		}
	}
}


void QTree::applyVisitor(QTreeVisitor& visitor){
	if ( root && visitor.accept(root) ){
		applyVisitor(root,visitor);
	}
}


void QTree::applyVisitorBottomUp(QTreeItem* item, QTreeVisitor& v){
	QTreeNode* n = item->parent;
	while( n && v.acceptParent(n) ){
		n = n->parent;
	}
	if(n){
		applyVisitor(n,v);
	}
}

void QTree::clear(){
	
}

void QTree::remove(QTreeItem* item){
	if(item){
		QTreeNode* n = item->parent;
		if(n==0) return;
		n->items.remove(item);
		item->parent = 0;
		m_size--;
		if( isToClean(n) ){
			for(int i=0;i<4;i++){
				delete n->childs[i];
				n->childs[i] = 0;
			}
			n->isLeaf = true;
		}
	}
}

}
