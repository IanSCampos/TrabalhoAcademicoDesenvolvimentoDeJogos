#include "BehaviorTree.hpp"

SelectorNode::SelectorNode(std::vector<BTNodePtr> children)
    : m_children(std::move(children)) {}

BTResult SelectorNode::tick(const BTContext& context) const {
    // Left-to-right evaluation: succeeds at the first succeeding child
    // and carries that child's command; fails iff all children fail.
    for (const auto& child : m_children) {
        if (!child) {
            continue;
        }
        BTResult result = child->tick(context);
        if (result.success) {
            return result;
        }
    }
    return {false, ""};
}

SequenceNode::SequenceNode(std::vector<BTNodePtr> children)
    : m_children(std::move(children)) {}

BTResult SequenceNode::tick(const BTContext& context) const {
    // Left-to-right evaluation: fails at the first failing child;
    // succeeds iff all succeed. Carries the last non-empty command.
    std::string lastCommand;

    for (const auto& child : m_children) {
        if (!child) {
            return {false, ""};
        }
        BTResult result = child->tick(context);
        if (!result.success) {
            return {false, ""};
        }
        if (!result.command.empty()) {
            lastCommand = std::move(result.command);
        }
    }

    return {true, lastCommand};
}

ConditionNode::ConditionNode(std::string name)
    : m_name(std::move(name)) {}

BTResult ConditionNode::tick(const BTContext& context) const {
    // Succeeds iff the context contains m_name with a non-zero value.
    // Carries no command.
    for (const auto& [factName, factValue] : context) {
        if (factName == m_name) {
            return {factValue != 0, ""};
        }
    }
    return {false, ""};
}

ActionNode::ActionNode(std::string command)
    : m_command(std::move(command)) {}

BTResult ActionNode::tick(const BTContext& /*context*/) const {
    // Always succeeds and carries its command string.
    return {true, m_command};
}

BehaviorTree::BehaviorTree(BTNodePtr root)
    : m_root(std::move(root)) {}

std::string BehaviorTree::tick(const BTContext& context) const {
    if (!m_root) {
        return "";
    }
    BTResult result = m_root->tick(context);
    return result.success ? result.command : "";
}