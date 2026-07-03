import { h } from './dom.js';
import { PARTS } from '../catalogue/parts.js';

const CATEGORY_LABELS = {
  endEffector: 'End Effectors',
  inline: 'Inline Sensors',
  addon: 'Wrist Add-ons',
  world: 'World Objects',
  base: 'Base Options',
};

/** Left-hand catalogue: click a card to mount/unmount a part. */
export class CataloguePanel {
  constructor(container, manager) {
    this.manager = manager;
    this.cards = new Map();

    const body = h('div.cat-body');
    const byCat = {};
    for (const p of PARTS) (byCat[p.category] ??= []).push(p);
    for (const [cat, parts] of Object.entries(byCat)) {
      body.append(h('h3', { text: CATEGORY_LABELS[cat] ?? cat }));
      for (const p of parts) {
        const card = h('button.cat-card', { onclick: () => manager.toggle(p.id) },
          h('div.cat-card-head', {},
            h('span.cat-name', { text: p.name }),
            h('span.cat-badge', { text: 'mounted' }),
          ),
          h('div.cat-desc', { text: p.desc }),
          p.mass ? h('div.cat-mass', { text: `${p.mass} kg` }) : null,
        );
        this.cards.set(p.id, card);
        body.append(card);
      }
      if (cat === 'endEffector') {
        body.append(h('div.cat-hint', { text: 'End effectors are exclusive — selecting one replaces the current one.' }));
      }
    }

    const shell = h('div.cat-shell', {},
      h('header.cat-head', {},
        h('span', { text: 'Catalogue' }),
        h('button.cat-toggle', { text: '‹', onclick: () => container.classList.toggle('collapsed') }),
      ),
      body,
    );
    container.append(shell);

    manager.onChange(() => this.refresh());
    this.refresh();
  }

  refresh() {
    for (const [id, card] of this.cards) {
      card.classList.toggle('active', this.manager.has(id));
    }
  }
}
