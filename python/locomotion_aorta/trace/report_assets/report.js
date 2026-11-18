'use strict';
const filter = document.getElementById('filter');
if (filter) {
  filter.addEventListener('input', () => {
    const terms = filter.value.toLowerCase().split(/\s+/).filter(Boolean);
    for (const article of document.querySelectorAll('article[data-filter]')) {
      const text = article.dataset.filter.toLowerCase();
      article.hidden = !terms.every(term => text.includes(term));
    }
  });
}
