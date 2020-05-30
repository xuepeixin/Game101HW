#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
        masses.resize(num_nodes);
        for (int i = 0; i < num_nodes; i++) {
            auto position = start + (end - start) * i / (num_nodes - 1);
            masses[i] = new Mass(position, node_mass, false);
        }
        springs.resize(num_nodes-1);
        for (int i = 0; i <  num_nodes - 1; i++) {
            springs[i] = new Spring(masses[i], masses[i+1], k);
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            s->m1->forces = s->m1->forces + s->k * (s->m2->position - s->m1->position) / (s->m2->position - s->m1->position).norm() *
                            ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m2->forces = s->m2->forces + s->k * (s->m1->position - s->m2->position) / (s->m1->position - s->m2->position).norm() *
                            ((s->m1->position - s->m2->position).norm() - s->rest_length);                
        }
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces = m->forces + m->mass * gravity;
                // TODO (Part 2): Add global damping
                auto a = m->forces / m->mass;
                auto v1 = m->velocity + a * delta_t;

                auto x1 = m->position + v1 * delta_t;
                m->velocity = v1;
                m->position = x1;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            s->m1->forces = s->m1->forces + s->k * (s->m2->position - s->m1->position) / (s->m2->position - s->m1->position).norm() *
                            ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m2->forces = s->m2->forces + s->k * (s->m1->position - s->m2->position) / (s->m1->position - s->m2->position).norm() *
                            ((s->m1->position - s->m2->position).norm() - s->rest_length);                
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces = m->forces + m->mass * gravity;
                m->position = m->position + (1-0.00005)*(m->position - m->last_position) + m->forces / m->mass * delta_t * delta_t;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
                
            }
            m->forces = Vector2D(0, 0);

        }
    }
}
