//rqt plugin for tuning parameters
//Author: Sebastian Schüller

#include <pluginlib/class_list_macros.h>
#include <boost/iterator/iterator_concepts.hpp>

#include <config_server/Save.h>
#include <config_server/Load.h>
#include <config_server/parameterclient.h>

#include <ros/this_node.h>

#include "parametertuner.h"
#include "parameteritem.h"


namespace parametertuner
{

Parametertuner::Parametertuner()
 : m_useJoystick(false)
 , m_updateCounter(0)
{}

Parametertuner::~Parametertuner()
{
}


void Parametertuner::initPlugin(qt_gui_cpp::PluginContext& context)
{
	// Initialize config_server library with the correct node handle.
	// We cannot use getPrivateNodeHandle() here, as it returns something like
	// /parametertuner/Parametertuner_1 as namespace, which is not unique -.-
	// Instead, use the global node handle with the ROS node name, which is
	// unique up to the rqt process (which is what we want).
	m_nh = ros::NodeHandle(getNodeHandle(), ros::this_node::getName());
	config_server::ParameterClient::initialize(m_nh);

	QWidget* w = new QWidget();

	m_ui.setupUi(w);
	
	qRegisterMetaType<std::string>("std::string");

	connect(m_ui.joystick_button, SIGNAL(toggled(bool)), this, SLOT(handleJoystickButton()));
	connect(m_ui.reset_button, SIGNAL(clicked(bool)), this, SLOT(reset()));
	connect(m_ui.save_button, SIGNAL(clicked(bool)), this, SLOT(save()));
	
	connect(m_ui.searchButton, SIGNAL(clicked(bool)), this, SLOT(handleSearchClicked()));
	connect(m_ui.searchInput, SIGNAL(returnPressed()), this, SLOT(handleEnterPressed()));

	connect(this, SIGNAL(updateRequested()), this, SLOT(update()), Qt::QueuedConnection);
	m_sub_paramList = getNodeHandle().subscribe("/config_server/parameter_list", 1, &Parametertuner::handleList, this);
	m_sub_joystick = getNodeHandle().subscribe("/joy", 1, &Parametertuner::handleJoystickInput, this);

	connect(this, SIGNAL(moveSelectionRequested(int)), this, SLOT(moveSelection(int)), Qt::QueuedConnection);
	connect(this, SIGNAL(valueChangeRequested(int)), this, SLOT(changeValue(int)), Qt::QueuedConnection);
	connect(this, SIGNAL(expansionRequested(int)), this, SLOT(handleExpansion(int)), Qt::QueuedConnection);

	m_ui.parameter_root_widget->setColumnCount(2);
	m_ui.parameter_root_widget->setColumnWidth(0, 260);

	context.addWidget(w);
	
	isSearchTurn = true;
	
	m_updateCounter = 0;
	m_updateTimer = m_nh.createWallTimer(ros::WallDuration(1.0), boost::bind(&Parametertuner::handleUpdateTimer, this), true, false);
}

void Parametertuner::shutdownPlugin()
{
	m_updateTimer.stop();
	m_sub_joystick.shutdown();
	m_sub_paramList.shutdown();
}

void Parametertuner::handleSearchClicked()
{
	if(isSearchTurn == true)
	{
		isSearchTurn = false;
		
		if(m_ui.searchInput->text().isEmpty())
		{
			m_ui.searchButton->setText("0 items");
			return;
		}
		
		search(m_ui.searchInput->text());
		m_ui.searchButton->setText(QString::number(itemsFound.size()) + " items");
	}
	else
	{
		isSearchTurn = true;
		
		m_ui.searchButton->setText("Search");
		unsearch();
	}
}

void Parametertuner::handleEnterPressed()
{
	m_ui.searchButton->click();
}

void Parametertuner::unsearch()
{
	// Collapse items which were found
	for(int i = 0; i < itemsFound.size(); i++)
		expandCollapseParents(itemsFound.at(i), false);
	
	// Clear current selection
	m_ui.parameter_root_widget->selectionModel()->clearSelection();
	
	restoreExpanded_and_Selected();
	itemsFound.clear();
}

void Parametertuner::search(QString text)
{
	itemsFound.clear();
	
	// Save current state of tree
	saveExpanded_and_Selected(m_ui.parameter_root_widget->invisibleRootItem());
	
	// Clear current selection
	m_ui.parameter_root_widget->selectionModel()->clearSelection();
	
	// Perform search
	itemsFound = m_ui.parameter_root_widget->findItems(text, Qt::MatchContains | Qt::MatchRecursive);
	
	// Show items which were found
	for(int i = 0; i < itemsFound.size(); i++)
	{
		expandCollapseParents(itemsFound.at(i), true);
		itemsFound.at(i)->setSelected(true);
	}
	
	// Scroll to first found item if exists
	if(itemsFound.size() > 0)
		m_ui.parameter_root_widget->scrollToItem(itemsFound.at(0));
}

void Parametertuner::saveBranches()
{
	QList<QTreeWidgetItem*> expanded;
	
	for(int i = 0; i < m_ui.parameter_root_widget->invisibleRootItem()->childCount(); i++)
		getExpandedItems(m_ui.parameter_root_widget->invisibleRootItem()->child(i), expanded);
	
	for(int i = 0; i < expanded.size(); i++)
	{
		branches.push_back(QVector<SavedTreeNode>());
		saveBranch(expanded.at(i), branches.last());
	}
}

void Parametertuner::restoreBranches()
{
	//printf("Restore\n");
	
	for(int i = 0; i < branches.size(); i++)
	{
		/*printf("Branch:\n");
		for(int j = 0; j < branches.at(i).size(); j++)
			printf("%s\n", branches.at(i).at(j).toStdString().c_str());
		printf("\n");*/
		
		QList<QTreeWidgetItem*> items;
		items = m_ui.parameter_root_widget->findItems(branches.at(i).last().name, Qt::MatchContains | Qt::MatchRecursive);
	
		for(int j = 0; j < items.size(); j++)
		{
			if(checkBranch(items.at(j), branches.at(i), branches.at(i).size()-1))
			{
				expandCollapseParents(items.at(j), true);
				restoreSelection(items.at(j), branches.at(i), branches.at(i).size()-1);
			}
		}
	}
}

void Parametertuner::restoreSelection(QTreeWidgetItem* item, const QVector< SavedTreeNode >& branch, int index)
{
	if(!item)
		return;

	if(index < 0)
		return;

	item->setSelected(branch.at(index).selected);
	restoreSelection(item->parent(), branch, --index);
}

bool Parametertuner::checkBranch(QTreeWidgetItem* item, const QVector<SavedTreeNode> &branch, int index)
{
	if(!item)
		return false;
	
	if(index < 0)
		return false;
	
	if(item->text(0) != branch.at(index).name)
		return false;
	
	if(index == 0)
		return true;
	else
		return checkBranch(item->parent(), branch, --index);
}

void Parametertuner::saveBranch(QTreeWidgetItem* item, QVector<SavedTreeNode> &branch)
{
	if(!item)
		return;
	
	SavedTreeNode node;
	node.name = item->text(0);
	node.selected = item->isSelected();
	
	branch.push_front(node);
	saveBranch(item->parent(), branch);
}

void Parametertuner::getExpandedItems(QTreeWidgetItem* item, QList<QTreeWidgetItem*> &list)
{
	if(!item)
		return;
	
	if(item->isExpanded() == false)
		return;
	
	bool noExpanded = true;
	
	for(int i = 0; i < item->childCount(); i++)
	{
		if(item->child(i)->isExpanded())
			noExpanded = false;
	}
	
	if(noExpanded)
		list.push_back(item);
	else
	{
		for(int i = 0; i < item->childCount(); i++)
			getExpandedItems(item->child(i), list);
	}
}

void Parametertuner::restoreExpanded_and_Selected()
{
	// Restore expanded items
	for(int i = 0; i < itemsExpanded.size(); i++)
		expandCollapseParents(itemsExpanded.at(i), true);
	
	// Restore selection
	for(int i = 0; i < itemsSelected.size(); i++)
		m_ui.parameter_root_widget->setItemSelected(itemsSelected.at(i), true);
	
	// Scroll to selection if exist
	if(itemsSelected.size() > 0)
		m_ui.parameter_root_widget->scrollToItem(itemsSelected.at(0));
	
	itemsExpanded.clear();
	itemsSelected.clear();
}

void Parametertuner::saveExpanded_and_Selected(QTreeWidgetItem *item)
{
	if(!item)
		return;
	
	if(item->isExpanded())
		itemsExpanded.push_back(item);
	
	if(item->isSelected())
		itemsSelected.push_back(item);
	
	for(int i = 0; i < item->childCount(); i++)
		saveExpanded_and_Selected(item->child(i));
}

void Parametertuner::expandCollapseParents(QTreeWidgetItem *item, bool expand)
{
	if(!item)
		return;
	
	if(expand)
		m_ui.parameter_root_widget->expandItem(item);
	else
		m_ui.parameter_root_widget->collapseItem(item);
	
	if(item->parent() == NULL)
		return;
	
	expandCollapseParents(item->parent(), expand);
}

void Parametertuner::handleList(const config_server::ParameterListConstPtr& list)
{
	QMutexLocker locker(&m_mutex);
	m_list = list;
	m_updateCounter++;
	if(m_updateCounter >= 20)
	{
		emit updateRequested();
		m_updateCounter = 0;
	}
	else
	{
		m_updateTimer.stop();
		m_updateTimer.start();
	}
}

void Parametertuner::handleUpdateTimer()
{
	m_updateTimer.stop(); // Required!
	emit updateRequested();
}

void Parametertuner::deleteWidget(QTreeWidgetItem* item)
{
	QWidget* w = m_ui.parameter_root_widget->itemWidget(item, 1);
	m_ui.parameter_root_widget->removeItemWidget(item, 1);

	if(w)
		delete w;

	if(!item)
		return;

	for(int i = 0; i < item->childCount(); ++i)
		deleteWidget(item->child(i));
}

void Parametertuner::update()
{
	branches.clear();
	
	itemsExpanded.clear();
	itemsSelected.clear();
	itemsFound.clear();
	
	m_ui.searchButton->setText("Search");
	isSearchTurn = true;
	
	// Save current state
	saveBranches();
	
	deleteWidget(m_ui.parameter_root_widget->invisibleRootItem());
	m_ui.parameter_root_widget->clear();

	config_server::ParameterClient* client = config_server::ParameterClient::instance();

	if(client)
		client->cork();

	QMutexLocker locker (&m_mutex);
	for(size_t i = 0; i < m_list->parameters.size(); i++)
	{
		QString param = QString::fromStdString(m_list->parameters[i].name);
		insertParameter(param, 0, m_list->parameters[i]);
	}

	if(client)
		client->uncork();
	
	// Restore saved state
	restoreBranches();
}

void Parametertuner::insertParameter(QString param, QTreeWidgetItem* ancestor, const config_server::ParameterDescription& description)
{
	if (param.isEmpty())
		return;

	QString levelParam, strippedParam;
	bool isLeaf;
	levelParam = param.section("/", 1, 1);
	strippedParam = param.section("/", 2);
	isLeaf = strippedParam.isEmpty();
	if (!isLeaf)
		strippedParam = "/" + strippedParam;


	QTreeWidgetItem* nextItem = 0;
	if (!ancestor)
	{
		QList<QTreeWidgetItem *> topLevelItems;
		topLevelItems = m_ui.parameter_root_widget->findItems(levelParam, 0);
		if (topLevelItems.isEmpty())
		{
			QTreeWidgetItem* newItem = 0;
			newItem = new QTreeWidgetItem();
			newItem->setText(0, levelParam);
			m_ui.parameter_root_widget->addTopLevelItem(newItem);
			nextItem  = newItem;
		}
		else
		{
			nextItem = topLevelItems.at(0);
		}
		insertParameter(strippedParam, nextItem, description);
		return;
	}

	int childCount = ancestor->childCount();
	int i;
	for (i = 0; i < childCount; i++)
	{
		if (levelParam == ancestor->child(i)->text(0))
		{
			insertParameter(strippedParam, ancestor->child(i), description);
			return;
		}
	}

	if (isLeaf)
		nextItem = insertParameterNode(description, ancestor);
	else
		nextItem = new QTreeWidgetItem(ancestor);

	nextItem->setText(0, levelParam);
	insertParameter(strippedParam, nextItem, description);
	return;

}

QTreeWidgetItem* Parametertuner::insertParameterNode(const config_server::ParameterDescription& description, QTreeWidgetItem* ancestor)
{
	QTreeWidgetItem* newItem = new QTreeWidgetItem(ancestor);

	QWidget* newWidget = 0;
	QString type = QString::fromStdString(description.type);
	if (type == "int")
		newWidget = new IntParameterWidget(getNodeHandle(), description);
	else if (type == "string")
		newWidget = new StringParameterWidget(getNodeHandle(), description);
	else if (type == "float")
		newWidget = new FloatParameterWidget(getNodeHandle(), description);
	else if (type == "bool")
		newWidget = new BoolParameterWidget(getNodeHandle(), description);

	if (newWidget != 0)
		m_ui.parameter_root_widget->setItemWidget(newItem, 1, newWidget);

	return newItem;
}

void Parametertuner::save()
{
	config_server::Save srv;

	if (!ros::service::call("/config_server/save",srv))
	{
		m_ui.save_button->setText("Saving failed");
		ROS_WARN("Could not call config_server service 'save'!");
		return;
	}
	m_ui.save_button->setText("Save Config");
}

void Parametertuner::reset()
{
	config_server::Load srv;
	srv.request.filename = "";

	if(!ros::service::call("/config_server/load", srv))
	{
		m_ui.reset_button->setText("Reset Failed");
		ROS_WARN("Could not call config_server service 'load'!");
	}
	m_ui.reset_button->setText("Reset Config");
}

void Parametertuner::handleJoystickButton()
{
	m_useJoystick = m_ui.joystick_button->isChecked();
}

void Parametertuner::handleJoystickInput(const sensor_msgs::JoyConstPtr& joy)
{
	if(!m_useJoystick)
		return;

	if(joy->buttons.size() < 8)
		return;

	int buttonUp = 4;
	int buttonDown = 5;
	int buttonLeft = 6;
	int buttonRight = 7;

	bool collapse = false;
	bool pressedUpLeft = joy->buttons[buttonUp] && joy->buttons[buttonLeft];
	if(m_buttonUpLeft && !pressedUpLeft)
	{
		expansionRequested(COLLAPSE);
		collapse = true;
	}
	m_buttonUpLeft = pressedUpLeft;

	if(m_buttonUp && !joy->buttons[buttonUp] && !collapse)
		moveSelectionRequested(UP);
	m_buttonUp = joy->buttons[buttonUp];

	if(m_buttonDown && !joy->buttons[buttonDown])
		moveSelectionRequested(DOWN);
	m_buttonDown = joy->buttons[buttonDown];

	if(m_buttonDec && !joy->buttons[buttonLeft] && !collapse)
		valueChangeRequested(LEFT);
	m_buttonDec = joy->buttons[buttonLeft];

	if(m_buttonInc && !joy->buttons[buttonRight])
		valueChangeRequested(RIGHT);
	m_buttonInc = joy->buttons[buttonRight];

}

// Get the parent of a tree item, including if it is a top level or root item already
QTreeWidgetItem* Parametertuner::itemGetParent(const QTreeWidgetItem* item)
{
	if(!item)
		return m_ui.parameter_root_widget->invisibleRootItem();
	QTreeWidgetItem* parentItem = item->parent();
	if(!parentItem)
		parentItem = m_ui.parameter_root_widget->invisibleRootItem();
	return parentItem;
}

// Get whether an item has a parent tree item (i.e. is not top level or root)
bool Parametertuner::itemHasParent(const QTreeWidgetItem* item)
{
	if(!item)
		return false;
	
	// If an item has no parent then NULL is returned, evaluating to a return value of false...
	return item->parent();
}

// Move the selection within the parameter tuner tree widget
void Parametertuner::moveSelection(int dir)
{
	// Error checking
	if(dir != UP && dir != DOWN) return;

	// Declare variables
	QTreeWidgetItem* item;
	QTreeWidgetItem* tmpItem;
	QTreeWidgetItem* thisItem;
	QTreeWidgetItem* parentItem;
	int itemIndex;

	// Get the currently selected item
	thisItem = getSelectedItem();

	// If no item is selected then select the first top level item
	if(!thisItem)
	{
		if(dir == DOWN)
			updateSelection(NULL, m_ui.parameter_root_widget->topLevelItem(0));
		else
			updateSelection(NULL, m_ui.parameter_root_widget->topLevelItem(m_ui.parameter_root_widget->topLevelItemCount() - 1));
		return;
	}

	// Handle UP case
	if(dir == UP)
	{
		// Go to the previous item in the tree
		parentItem = itemGetParent(thisItem);
		if(!parentItem)
			return;
		itemIndex = parentItem->indexOfChild(thisItem);
		tmpItem = parentItem->child(itemIndex + UP);
		if(tmpItem)
		{
			item = tmpItem;
			while(item->childCount() > 0 && item->isExpanded())
				item = item->child(item->childCount() - 1);
			updateSelection(thisItem, item);
			return;
		}
		else if(itemHasParent(thisItem))
		{
			updateSelection(thisItem, parentItem);
			return;
		}
		else return; // <-- This is the case that we are at the very top-most element
	}

	// Handle DOWN case
	if(dir == DOWN)
	{
		// If the item has children and is expanded then go to the first child
		if(thisItem->childCount() > 0 && thisItem->isExpanded())
		{
			updateSelection(thisItem, thisItem->child(0));
			return;
		}

		// Go to the next item in the tree
		item = thisItem;
		while(true)
		{
			parentItem = itemGetParent(item);
			if(!parentItem)
				return;
			itemIndex = parentItem->indexOfChild(item);
			tmpItem = parentItem->child(itemIndex + DOWN);
			if(tmpItem)
			{
				updateSelection(thisItem, tmpItem);
				return;
			}
			else
			{
				if(itemHasParent(item))
					item = parentItem;
				else return; // <-- This is the case that we are at the very bottom-most element
			}
		}
	}
}

QTreeWidgetItem* Parametertuner::getSelectedItem()
{
	QList<QTreeWidgetItem *> selectList = m_ui.parameter_root_widget->selectedItems();
	if (selectList.size() == 0)
	{
		return 0;
	}
	else
		return selectList.at(0);
}

void Parametertuner::updateSelection(QTreeWidgetItem* old, QTreeWidgetItem* next)
{
	if(old) old->setSelected(false);
	if(next)
	{
		next->setSelected(true);
		m_ui.parameter_root_widget->scrollToItem(next);
	}
}

void Parametertuner::changeValue(int dir)
{
	QTreeWidgetItem* thisItem;
	if (!(thisItem = getSelectedItem()))
		return;

	if (thisItem->childCount() != 0)
	{
		if (dir == RIGHT && (!thisItem->isExpanded()))
			thisItem->setExpanded(true);
		else if (dir == LEFT && thisItem->isExpanded())
			thisItem->setExpanded(false);
		return;
	}
	ParameterWidgetBase* param;
	param = (ParameterWidgetBase *)m_ui.parameter_root_widget->itemWidget(thisItem, 1);
	if (!param)
		return;

	if (dir == LEFT)
		param->DecValue();
	else
		param->IncValue();
}

void Parametertuner::handleExpansion(int dir)
{
	// Error checking
	if(dir != COLLAPSE) return;

	// Declare variables
	QTreeWidgetItem* thisItem;
	QTreeWidgetItem* parentItem;
	
	// Get the currently selected item
	thisItem = getSelectedItem(); // Careful: This may still be null!
	
	// Handle case where collapse has been requested
	if(dir == COLLAPSE)
	{
		// Can't collapse the whole root
		if(!thisItem) return;
		if(!itemHasParent(thisItem)) return;

		// Collapse the parent item of the currently selected item
		parentItem = itemGetParent(thisItem);
		updateSelection(thisItem, parentItem);
		parentItem->setExpanded(false);

		// Done
		return;
	}
}

}

PLUGINLIB_EXPORT_CLASS(parametertuner::Parametertuner, rqt_gui_cpp::Plugin)
// EOF