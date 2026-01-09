export class ContextCapture {
  /**
   * Gets the currently selected text on the page
   * @returns The selected text or empty string if nothing is selected
   */
  static getSelectedText(): string {
    if (typeof window !== 'undefined') {
      return window.getSelection()?.toString() || '';
    }
    return '';
  }

  /**
   * Gets the current page URL
   * @returns The current page URL
   */
  static getCurrentPageUrl(): string {
    if (typeof window !== 'undefined') {
      return window.location.href;
    }
    return '';
  }

  /**
   * Gets the current page title
   * @returns The current page title
   */
  static getCurrentPageTitle(): string {
    if (typeof document !== 'undefined') {
      return document.title;
    }
    return '';
  }

  /**
   * Gets the current page metadata
   * @returns An object containing URL, title, and other relevant metadata
   */
  static getCurrentPageMetadata(): {
    url: string;
    title: string;
    description?: string;
    section?: string;
  } {
    const metadata: {
      url: string;
      title: string;
      description?: string;
      section?: string;
    } = {
      url: ContextCapture.getCurrentPageUrl(),
      title: ContextCapture.getCurrentPageTitle(),
    };

    // Try to get description from meta tag
    if (typeof document !== 'undefined') {
      const descriptionEl = document.querySelector('meta[name="description"]');
      if (descriptionEl) {
        metadata.description = descriptionEl.getAttribute('content') || '';
      }

      // Try to infer section from URL or page structure
      const pathParts = new URL(metadata.url).pathname.split('/').filter(part => part);
      if (pathParts.length > 0) {
        metadata.section = pathParts[pathParts.length - 1];
      }
    }

    return metadata;
  }

  /**
   * Captures the current context including selected text and page metadata
   * @returns An object with context information
   */
  static captureContext(): {
    selectedText: string;
    currentPageUrl: string;
    currentPageTitle: string;
    currentPageDescription?: string;
    currentPageSection?: string;
  } {
    const pageMetadata = ContextCapture.getCurrentPageMetadata();
    const selectedText = ContextCapture.getSelectedText();

    return {
      selectedText,
      currentPageUrl: pageMetadata.url,
      currentPageTitle: pageMetadata.title,
      currentPageDescription: pageMetadata.description,
      currentPageSection: pageMetadata.section,
    };
  }

  /**
   * Gets text from a specific element on the page
   * @param selector - CSS selector for the element
   * @returns The text content of the element or empty string if not found
   */
  static getTextFromElement(selector: string): string {
    if (typeof document !== 'undefined') {
      const element = document.querySelector(selector);
      return element ? element.textContent || '' : '';
    }
    return '';
  }

  /**
   * Gets all text content from the main content area of the page
   * @param selectors - Array of potential selectors for main content areas
   * @returns The text content from the main content area
   */
  static getMainContent(selectors: string[] = ['.main-content', 'main', '.container', '#content']): string {
    if (typeof document !== 'undefined') {
      for (const selector of selectors) {
        const element = document.querySelector(selector);
        if (element) {
          return element.textContent || '';
        }
      }
      // If no specific selector matches, return the body text
      return document.body.textContent || '';
    }
    return '';
  }

  /**
   * Captures text selection and surrounding context
   * @param contextWords - Number of words before and after selection to include
   * @returns Object with selection and surrounding context
   */
  static captureSelectionWithContext(contextWords: number = 50): {
    selectedText: string;
    contextBefore: string;
    contextAfter: string;
    fullContext: string;
  } {
    const selectedText = ContextCapture.getSelectedText();

    if (!selectedText) {
      return {
        selectedText: '',
        contextBefore: '',
        contextAfter: '',
        fullContext: ''
      };
    }

    // Get the main content to extract context
    const mainContent = ContextCapture.getMainContent();
    const selectionIndex = mainContent.indexOf(selectedText);

    if (selectionIndex === -1) {
      return {
        selectedText,
        contextBefore: '',
        contextAfter: '',
        fullContext: selectedText
      };
    }

    // Extract context before the selection
    const beforeStart = Math.max(0, selectionIndex - contextWords * 10); // Rough estimate of characters
    const contextBefore = mainContent.substring(beforeStart, selectionIndex).trim();

    // Extract context after the selection
    const afterEnd = Math.min(mainContent.length, selectionIndex + selectedText.length + contextWords * 10);
    const contextAfter = mainContent.substring(selectionIndex + selectedText.length, afterEnd).trim();

    const fullContext = contextBefore + ' [SELECTED TEXT: ' + selectedText + '] ' + contextAfter;

    return {
      selectedText,
      contextBefore,
      contextAfter,
      fullContext
    };
  }
}